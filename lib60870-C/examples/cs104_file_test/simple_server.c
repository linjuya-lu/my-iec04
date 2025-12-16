#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <math.h>

#include "cs104_slave.h"
#include "hal_thread.h"
#include "hal_time.h"
#include "iec60870_common.h"  

static bool running = true;
static volatile bool g_needSendWriteAct = false;
static IMasterConnection g_conn = NULL;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void
sigint_handler(int signalId)
{
    (void)signalId;
    running = false;
}

/* -------------------- 文件发送相关状态 -------------------- */

#define FILE_IOA       5000      /* 文件服务的 IOA，随便约定一个 */
#define FILE_ID_DEMO   1         /* 测试文件 ID */
#define FILE_SEG_SIZE  200       /* 每帧最多发送的数据字节数 */

typedef enum {
    FILE_STATE_IDLE = 0,
    FILE_STATE_WAIT_ACT_ACK,
    FILE_STATE_WAIT_DATA_ACK,
    FILE_STATE_DONE
} FileTxState;

static CS101_AppLayerParameters g_alParams = NULL;

typedef struct {
    bool     prepared;
    uint32_t fileId;
    uint32_t fileSize;
    uint32_t segSize;
    uint8_t *data;
} FileTxCtx;

static FileTxCtx  g_fileTx;
static FileTxState g_fileState = FILE_STATE_IDLE;

/* 简单“单字节求和取低 8 位”的校验算法 */
static uint8_t
calcChecksum(const uint8_t *buf, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)sum;
}

/* 准备一份测试文件数据：1024 字节的简易正弦波 */
static void
prepareTestFile(void)
{
    if (g_fileTx.prepared)
        return;

    int len = 1024;
    uint8_t *buf = (uint8_t *)malloc((size_t)len);
    if (!buf) {
        printf("[FILE] malloc failed\n");
        return;
    }

    int16_t *p = (int16_t *)buf;
    int samples = len / 2;

    for (int i = 0; i < samples; i++) {
        double ang = (double)i / (double)samples * 2.0 * M_PI;
        int16_t v = (int16_t)(20000.0 * sin(ang));  /* -20000 ~ 20000 */
        p[i] = v;                                   /* 小端机器：低字节在前 */
    }

    g_fileTx.prepared = true;
    g_fileTx.fileId   = FILE_ID_DEMO;
    g_fileTx.fileSize = (uint32_t)len;
    g_fileTx.segSize  = FILE_SEG_SIZE;
    g_fileTx.data     = buf;

    printf("[FILE] prepared test file: size=%u bytes\n", g_fileTx.fileSize);
}

/* 发送：写文件激活（op=7，COT=6） */
static void
sendWriteActivate(IMasterConnection connection)
{
    prepareTestFile();
    if (!g_fileTx.prepared) {
        printf("[FILE] prepareTestFile failed\n");
        return;
    }

    const char *fileName = "demo.bin";
    uint8_t nameLen = (uint8_t)strlen(fileName);

    CS101_ASDU asdu = CS101_ASDU_create(
        g_alParams,
        false,
        CS101_COT_ACTIVATION, /* 6：写文件激活 */
        0,
        1,                     /* 公共地址 */
        false,
        false
    );

    FileExt210 fe = FileExt210_createWriteAct(
        FILE_IOA,
        (const uint8_t *)fileName,
        nameLen,
        g_fileTx.fileId,
        g_fileTx.fileSize
    );

    CS101_ASDU_addInformationObject(asdu, (InformationObject)fe);

    printf("[FILE] send WRITE_ACT (op=7) fileId=%u size=%u\n",
           g_fileTx.fileId, g_fileTx.fileSize);

    bool ok = IMasterConnection_sendASDU(connection, asdu);
    printf("[FILE] send WRITE_ACT ok=%d\n", ok);
    InformationObject_destroy((InformationObject)fe);
    CS101_ASDU_destroy(asdu);

    g_fileState = FILE_STATE_WAIT_ACT_ACK;
}

/* 发送：文件数据（op=9，COT=5） */
static void
sendWriteDataAll(IMasterConnection connection)
{
    if (!g_fileTx.prepared || g_fileTx.data == NULL) {
        printf("[FILE] no file to send\n");
        return;
    }

    uint32_t offset = 0;
    uint32_t segNo  = 0;

    while (offset < g_fileTx.fileSize) {
        int chunk = (int)g_fileTx.segSize;
        if (offset + (uint32_t)chunk > g_fileTx.fileSize)
            chunk = (int)(g_fileTx.fileSize - offset);

        uint8_t hasMore = (offset + (uint32_t)chunk < g_fileTx.fileSize) ? 1 : 0;
        uint8_t checksum = calcChecksum(g_fileTx.data + offset, chunk);

        CS101_ASDU asdu = CS101_ASDU_create(
            g_alParams,
            false,
            CS101_COT_REQUEST,   /* 5：写文件数据 */
            0,
            1,
            false,
            false
        );

        FileExt210 fe = FileExt210_createWriteData(
            FILE_IOA,
            g_fileTx.fileId,
            segNo,
            hasMore,
            g_fileTx.data + offset,
            chunk,
            checksum
        );

        CS101_ASDU_addInformationObject(asdu, (InformationObject)fe);

        printf("[FILE] send WRITE_DATA (op=9) seg=%u len=%d hasMore=%d checksum=0x%02x\n",
               segNo, chunk, hasMore, checksum);

        IMasterConnection_sendASDU(connection, asdu);

        InformationObject_destroy((InformationObject)fe);
        CS101_ASDU_destroy(asdu);

        offset += (uint32_t)chunk;
        segNo++;
    }

    g_fileState = FILE_STATE_WAIT_DATA_ACK;
}

/* -------------------- 其他示例回调（保留原来的） -------------------- */

void
printCP56Time2a(CP56Time2a time)
{
    printf("%02i:%02i:%02i %02i/%02i/%04i",
           CP56Time2a_getHour(time),
           CP56Time2a_getMinute(time),
           CP56Time2a_getSecond(time),
           CP56Time2a_getDayOfMonth(time),
           CP56Time2a_getMonth(time),
           CP56Time2a_getYear(time) + 2000);
}

/* 原始报文打印 */
static void
rawMessageHandler(void* parameter, IMasterConnection connection, uint8_t* msg, int msgSize, bool sent)
{
    (void)parameter;
    (void)connection;

    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    for (int i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }
    printf("\n");
}

static bool
clockSyncHandler (void* parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
    (void)parameter;
    (void)connection;
    (void)asdu;

    printf("Process time sync command with time ");
    printCP56Time2a(newTime);
    printf("\n");
    return true;
}

/* 总召处理逻辑：沿用官方示例 */
static bool
interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
    (void)parameter;
    printf("Received interrogation for group %i\n", qoi);
    if (qoi == 20) /* 只处理站总召 */
    {
        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
        IMasterConnection_sendACT_CON(connection, asdu, false);

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                0, 1, false, false);
        InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, 100, -1, IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            MeasuredValueScaled_create((MeasuredValueScaled) io, 101, 23, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            MeasuredValueScaled_create((MeasuredValueScaled) io, 102, 2300, IEC60870_QUALITY_GOOD));
        InformationObject_destroy(io);
        IMasterConnection_sendASDU(connection, newAsdu);
        CS101_ASDU_destroy(newAsdu);
        newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                    0, 1, false, false);
        io = (InformationObject) SinglePointInformation_create(NULL, 104, true, IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            SinglePointInformation_create((SinglePointInformation) io, 105, false, IEC60870_QUALITY_GOOD));
        InformationObject_destroy(io);
        IMasterConnection_sendASDU(connection, newAsdu);
        CS101_ASDU_destroy(newAsdu);

        IMasterConnection_sendACT_TERM(connection, asdu);
    }
    else
    {
        IMasterConnection_sendACT_CON(connection, asdu, true);
    }
    return true;
}

/* --------------------文件服务&遥控-------------------- */
static bool
asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    (void)parameter;
    TypeID ti  = CS101_ASDU_getTypeID(asdu);
    CS101_CauseOfTransmission  cot = CS101_ASDU_getCOT(asdu);

    /*文件服务*/
    if (ti == M_FT_EXT_1) {
        printf("[FILE] recv TI=210 COT=%d\n", cot);
        /*只关心确认（COT=7） */
        if (cot == CS101_COT_ACTIVATION_CON) {
            if (g_fileState == FILE_STATE_WAIT_ACT_ACK) {
                printf("[FILE] got WRITE_ACT_ACK, start sending data...\n");
                sendWriteDataAll(connection);
            }
            else if (g_fileState == FILE_STATE_WAIT_DATA_ACK) {
                printf("[FILE] got WRITE_DATA_ACK, file transfer done.\n");
                g_fileState = FILE_STATE_DONE;
            }
            else {
                printf("[FILE] unexpected ACTIVATION_CON in state=%d\n", g_fileState);
            }
        }

        InformationObject io = CS101_ASDU_getElement(asdu, 0);
        if (io)
            InformationObject_destroy(io);

        return true;
    }

    /*其他类型：单点遥控*/
    if (ti == C_SC_NA_1) {
        printf("received single command\n");

        if  (cot == CS101_COT_ACTIVATION) {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);

            if (io) {
                if (InformationObject_getObjectAddress(io) == 5000) {
                    SingleCommand sc = (SingleCommand) io;

                    printf("IOA: %i switch to %i\n",
                           InformationObject_getObjectAddress(io),
                           SingleCommand_getState(sc));

                    CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                }
                else
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);

                InformationObject_destroy(io);
            }
            else {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);

        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }

    return false;
}

/* -------------------- 连接事件回调 -------------------- */

static bool connected = false;

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
    (void)parameter;
    printf("New connection request from %s\n", ipAddress);
    return true;
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    (void)parameter;

    if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);

        g_conn = con;
        g_needSendWriteAct = true;   
    }
}


int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    // signal(SIGINT, sigint_handler);
    CS104_Slave slave = CS104_Slave_create(10, 10);

    CS104_Slave_setLocalAddress(slave, "0.0.0.0");

    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);
    g_alParams = alParams;

    CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);

    printf("APCI parameters:\n");
    printf("  t0: %i\n", apciParams->t0);
    printf("  t1: %i\n", apciParams->t1);
    printf("  t2: %i\n", apciParams->t2);
    printf("  t3: %i\n", apciParams->t3);
    printf("  k: %i\n", apciParams->k);
    printf("  w: %i\n", apciParams->w);

    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);
    printf("raw handler set = %p\n", rawMessageHandler);
    
    memset(&g_fileTx, 0, sizeof(g_fileTx));
    g_fileState = FILE_STATE_IDLE;

    CS104_Slave_start(slave);

    if (!CS104_Slave_isRunning(slave)) {
        printf("Starting server failed!\n");
        CS104_Slave_destroy(slave);
        return -1;
    }

    printf("Server started, press Ctrl+C to stop.\n");

    int16_t scaledValue = 0;

    while (running) {
        Thread_sleep(1000);

        if (g_needSendWriteAct) {
            g_needSendWriteAct = false;
            sendWriteActivate(g_conn);   // ✅ 在主线程发
        }
        
        /* 保留原示例的一个周期量测，方便你在终端工具里看到数值变化 */
        CS101_ASDU newAsdu = CS101_ASDU_create(
            alParams,
            false,
            CS101_COT_PERIODIC,
            0,
            1,
            false,
            false
        );

        InformationObject io = (InformationObject)
            MeasuredValueScaled_create(NULL, 110, scaledValue, IEC60870_QUALITY_GOOD);

        scaledValue++;

        CS101_ASDU_addInformationObject(newAsdu, io);
        InformationObject_destroy(io);

        CS104_Slave_enqueueASDU(slave, newAsdu);
        CS101_ASDU_destroy(newAsdu);
    }

    printf("Stopping server\n");
    CS104_Slave_stop(slave);

    if (g_fileTx.data) {
        free(g_fileTx.data);
        g_fileTx.data = NULL;
    }

    CS104_Slave_destroy(slave);
    Thread_sleep(500);

    return 0;
}
