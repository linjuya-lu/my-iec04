#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdint.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include "cs104_slave.h"
#include "hal_thread.h"
#include "iec60870_common.h"
#include "cs101_slave.h"
#ifndef FILE_IOA
#define FILE_IOA 0               
#endif
#define FILE_DIR_BASE "/home/water/edgex-v4/device-server/IEC104/lib60870/lib60870-C/examples/cs104_file_server_test/Emd/data/COMTRADE"

#define FILE_CHUNK    200        
#define MAX_DIR_FILES 16

#define OP_DIR_CALL        1  // 读目录
#define OP_DIR_CALL_CON    2  // 读目录确认
#define OP_READ_ACT        3  // 读文件激活
#define OP_READ_ACT_ACK    4  // 读文件激活确认
#define OP_READ_DATA       5  // 读文件数据
#define OP_READ_DATA_CON   6  // 读文件数据确认

//全局
static volatile bool g_running = true;
static volatile bool g_dirSending = false;
static CS101_AppLayerParameters g_alParams = NULL;
static IMasterConnection g_conn = NULL;
static uint8_t g_dirResult = 0;   // 0成功 1失败
static time_t g_dirMtime[MAX_DIR_FILES];


//目录
static char     g_dirFiles[MAX_DIR_FILES][128];
static uint32_t g_dirSizes[MAX_DIR_FILES];
static int      g_dirCount = 0;
static int      g_dirIndex = 0;
static volatile bool g_needSendDirAck = false;
static uint32_t g_dirId = 0;

//文件
typedef struct {
    bool     loaded;
    bool     active;     //是否开始发数据
    uint32_t fileId;
    uint32_t fileSize;
    uint32_t offset;
    char     fileName[256];
    uint8_t* data;
} ReadTxCtx;
static ReadTxCtx g_readTx;
static volatile bool g_needSendReadActAck = false;

static void fill_cp56time2a_from_time_t(time_t t, uint8_t out[7])
{
    struct tm tmv;
    localtime_r(&t, &tmv);   //本地时区
    // CP56Time2a: ms(2 LE), min, hour, day(+wday), mon, year
    uint16_t ms = (uint16_t)(tmv.tm_sec * 1000); // 没有毫秒就用 sec*1000

    out[0] = (uint8_t)(ms & 0xFF);
    out[1] = (uint8_t)((ms >> 8) & 0xFF);
    out[2] = (uint8_t)(tmv.tm_min & 0x3F);   // 0..59
    out[3] = (uint8_t)(tmv.tm_hour & 0x1F);  // 0..23

    // day of week: CP56Time2a 通常 1=Mon..7=Sun
    int wday = tmv.tm_wday;     // 0=Sun..6=Sat
    int cp_wday = (wday == 0) ? 7 : wday;  // 1..7
    out[4] = (uint8_t)((tmv.tm_mday & 0x1F) | ((cp_wday & 0x07) << 5));
    out[5] = (uint8_t)((tmv.tm_mon + 1) & 0x0F);  // 1..12
    out[6] = (uint8_t)((tmv.tm_year % 100) & 0x7F); // 0..99
}

FileExt210
FileExt210_createDirCallAck(int                        ioa,
                            uint8_t                    result,
                            uint32_t                   dirId,
                            uint8_t                    hasMore,
                            uint8_t                    fileCount,
                            const File210_DirFileEntry *files);
static void on_sigint(int sig) 
{ 
    (void)sig; 
    g_running = false; 
}

static uint8_t checksum8(const uint8_t* p, uint32_t n)
{
    uint8_t s = 0;
    for (uint32_t i = 0; i < n; i++) s += p[i];
    return s;
}

static void buildDirList(const char* dirPath)
{
    printf("[FILE] buildDirList path=%s\n", dirPath);
    g_dirCount = 0;
    g_dirIndex = 0;
    g_dirResult = 0;
    DIR* d = opendir(dirPath);
    if (!d) {
        printf("[FILE] opendir failed: %s (%s)\n", dirPath, strerror(errno));
        g_dirResult = 1;               
        return;
    }
    struct dirent* ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        if (g_dirCount >= MAX_DIR_FILES) break;
        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dirPath, ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0) continue;
        if (!S_ISREG(st.st_mode)) continue;

        strncpy(g_dirFiles[g_dirCount], ent->d_name, sizeof(g_dirFiles[g_dirCount]) - 1);
        g_dirFiles[g_dirCount][sizeof(g_dirFiles[g_dirCount]) - 1] = '\0';
        g_dirSizes[g_dirCount] = (uint32_t)st.st_size;
        g_dirMtime[g_dirCount] = st.st_mtime;   
        g_dirCount++;
    }
    closedir(d);
    printf("[FILE] dir list ready: path=%s files=%d\n", dirPath, g_dirCount);
}

static void sendDirAckPage(IMasterConnection con)
{
    if (!con) return;

    /* ====== 关键修改：即使没有文件也要回一帧 ACK ====== */
    if (g_dirCount <= 0) {
        File210_DirFileEntry entries[FILE210_MAX_DIR_FILES];
        memset(entries, 0, sizeof(entries));

        uint8_t n = 0;
        uint8_t hasMore = 0;

        CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);
        FileExt210 io = FileExt210_createDirCallAck(
            FILE_IOA,
            g_dirResult,   /* 0成功 1失败（buildDirList里已经设了） */
            g_dirId,
            hasMore,
            n,
            entries
        );

        CS101_ASDU_addInformationObject(a, (InformationObject)io);
        bool ok = IMasterConnection_sendASDU(con, a);

        printf("[FILE] SEND DIR_CALL_ACK(EMPTY) dirId=%u result=%u n=%u hasMore=%u ok=%d\n",
               g_dirId, g_dirResult, n, hasMore, ok);

        CS101_ASDU_destroy(a);
        InformationObject_destroy((InformationObject)io);

        g_dirSending = false;
        return;
    }

    /* ====== 原逻辑：分页发送 ====== */
    if (g_dirIndex >= g_dirCount) {
        g_dirSending = false;
        return;
    }

    File210_DirFileEntry entries[FILE210_MAX_DIR_FILES];
    memset(entries, 0, sizeof(entries));

    int left = g_dirCount - g_dirIndex;
    uint8_t n = (left > FILE210_MAX_DIR_FILES) ? FILE210_MAX_DIR_FILES : (uint8_t)left;
    uint8_t hasMore = (g_dirIndex + n < g_dirCount) ? 1 : 0;

    for (uint8_t i = 0; i < n; i++) {
        const char* name = g_dirFiles[g_dirIndex + i];
        uint8_t nameLen = (uint8_t)strlen(name);
        if (nameLen > FILE210_MAX_NAME) nameLen = FILE210_MAX_NAME;
        entries[i].nameLen = nameLen;
        memcpy(entries[i].name, name, nameLen);
        entries[i].attr = 0;
        entries[i].size = g_dirSizes[g_dirIndex + i];
        fill_cp56time2a_from_time_t(g_dirMtime[g_dirIndex + i], entries[i].time);
    }

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);
    FileExt210 io = FileExt210_createDirCallAck(
        FILE_IOA,
        g_dirResult,
        g_dirId,
        hasMore,
        n,
        entries
    );

    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(con, a);

    printf("[FILE] SEND DIR_CALL_ACK dirId=%u result=%u n=%u hasMore=%u ok=%d\n",
           g_dirId, g_dirResult, n, hasMore, ok);

    CS101_ASDU_destroy(a);
    InformationObject_destroy((InformationObject)io);

    g_dirIndex += n;
    if (!hasMore) g_dirSending = false;
}

static void freeReadTx(void)
{
    if (g_readTx.data) {
        free(g_readTx.data);
        g_readTx.data = NULL;
    }
    memset(&g_readTx, 0, sizeof(g_readTx));
}
static void rawMessageHandler(void* parameter, IMasterConnection connection, uint8_t* msg, int msgSize, bool sent)
{
    (void)parameter;
    (void)connection;
    if (msg == NULL || msgSize <= 0)
        return;
    printf("%s (%d): ", sent ? "SEND" : "RCVD", msgSize);
    for (int i = 0; i < msgSize; i++)
        printf("%02x ", msg[i]);
    printf("\n");
    fflush(stdout);
}
static bool interrogationHandler(void* parameter, IMasterConnection connection,
                                 CS101_ASDU asdu, uint8_t qoi)
{
    (void)parameter;
    (void)qoi;
    //回确认+结束
    IMasterConnection_sendACT_CON(connection, asdu, false);
    IMasterConnection_sendACT_TERM(connection, asdu);
    return true;
}
static bool clockSyncHandler(void* parameter, IMasterConnection connection,
                            CS101_ASDU asdu, CP56Time2a newTime)
{
    (void)parameter;
    (void)newTime;
    IMasterConnection_sendACT_CON(connection, asdu, false);
    IMasterConnection_sendACT_TERM(connection, asdu);
    return true;
}

//读文件激活确认
static void sendReadActAck(IMasterConnection con)
{
    uint8_t result = (g_readTx.data && g_readTx.fileSize > 0) ? 0 : 1;

    uint8_t nameLen = (uint8_t)strlen(g_readTx.fileName);   // g_readTx.fileName 里是 '\0' 结尾的
    const uint8_t* name = (const uint8_t*)g_readTx.fileName;

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_ACTIVATION_CON, 0, 1, false, false);

    FileExt210 io = FileExt210_createReadActAck(
        FILE_IOA,
        result,
        name,
        nameLen,
        g_readTx.fileId,
        g_readTx.fileSize
    );
    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(con, a);
    printf("[FILE] SEND READ_ACT_ACK result=%u name=%s fileId=%u size=%u ok=%d\n",
           result, g_readTx.fileName, g_readTx.fileId, g_readTx.fileSize, ok);
    CS101_ASDU_destroy(a);
    InformationObject_destroy((InformationObject)io);

    if (result == 0) {
        g_readTx.offset = 0;
        g_readTx.active = true;
    }
}
static void pumpReadData(IMasterConnection con)
{
    if (!con || !g_alParams) return;
    if (!g_readTx.active) return;
    if (!g_readTx.data || g_readTx.fileSize == 0) { g_readTx.active = false; return; }

    if (g_readTx.offset >= g_readTx.fileSize) {
        printf("[FILE] READ_DATA all sent, wait op=6\n");
        g_readTx.active = false;
        return;
    }

    uint32_t left = g_readTx.fileSize - g_readTx.offset;
    uint32_t n = (left > FILE_CHUNK) ? FILE_CHUNK : left;

    uint32_t segNo = g_readTx.offset;                 //用偏移当段号
    uint8_t  hasMore = (g_readTx.offset + n < g_readTx.fileSize) ? 1 : 0;

    uint8_t* seg = (uint8_t*)malloc(n);
    if (!seg) { printf("[FILE] malloc seg failed\n"); return; }
    memcpy(seg, g_readTx.data + g_readTx.offset, n);

    uint8_t cks = checksum8(seg, n);

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);
    if (!a) { free(seg); return; }

    FileExt210 io = FileExt210_createReadData(FILE_IOA, g_readTx.fileId, segNo, hasMore, seg, (int)n, cks);
    if (!io) { CS101_ASDU_destroy(a); free(seg); return; }

    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(con, a);

    printf("[FILE] SEND READ_DATA fileId=%u segNo=%u len=%u hasMore=%u cks=%02x ok=%d\n",
           g_readTx.fileId, segNo, n, hasMore, cks, ok);

    CS101_ASDU_destroy(a);

    InformationObject_destroy((InformationObject)io);

    if (ok) g_readTx.offset += n;
}


static bool asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    (void)parameter;
    TypeID ti = CS101_ASDU_getTypeID(asdu);
    if ((int)ti != 210) return true;

    g_conn = connection;
    FileExt210 fe = (FileExt210)CS101_ASDU_getElement(asdu, 0);
    if (!fe) return true;
    uint8_t op = fe->op; 
    if (op == OP_DIR_CALL) {
        char dirName[128] = {0};
        uint8_t nameLen = fe->u.dirCall.nameLen;
        if (nameLen > sizeof(dirName) - 1) nameLen = sizeof(dirName) - 1;
        memcpy(dirName, fe->u.dirCall.name, nameLen);

        g_dirId = fe->u.dirCall.dirId;
        printf("[FILE] RCVD DIR_CALL: dirId=%u dir=%s\n", g_dirId, dirName[0] ? dirName : "(default)");
        buildDirList(FILE_DIR_BASE);
        g_dirIndex = 0;
        g_dirSending = true;
        sendDirAckPage(connection);
    } else if (op == OP_READ_ACT) {
        // 读文件激活：取文件名
        freeReadTx();
        uint8_t nameLen = fe->u.readAct.nameLen;
        if (nameLen > sizeof(g_readTx.fileName) - 1) nameLen = sizeof(g_readTx.fileName) - 1;

        memcpy(g_readTx.fileName, fe->u.readAct.name, nameLen);
        g_readTx.fileName[nameLen] = '\0';
        printf("[FILE] RCVD READ_ACT: file=%s\n", g_readTx.fileName);

        // 文件路径：/Emd/data/COMTRADE/<filename>
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", FILE_DIR_BASE, g_readTx.fileName);

        FILE* fp = fopen(path, "rb");
        if (!fp) {
            printf("[FILE] open failed: %s (%s)\n", path, strerror(errno));
            g_readTx.loaded = false;
        } else {
            fseek(fp, 0, SEEK_END);
            long sz = ftell(fp);
            fseek(fp, 0, SEEK_SET);

            if (sz > 0) {
                g_readTx.data = (uint8_t*)malloc((size_t)sz);
                if (g_readTx.data) {
                    size_t rd = fread(g_readTx.data, 1, (size_t)sz, fp);
                    g_readTx.fileSize = (uint32_t)rd;
                    g_readTx.fileId = 1;   // 这里先固定 1；需要的话可做递增分配
                    g_readTx.loaded = true;
                }
            }
            fclose(fp);
        }

        g_needSendReadActAck = true;

    } else if (op == OP_READ_DATA_CON) {
        // 主站确认：读文件结束
        printf("[FILE] RCVD READ_DATA_CON: transfer done\n");
        freeReadTx();
    } else {
        printf("[FILE] RCVD TI=210 op=%u (ignored)\n", op);
    }
    return true;
}

static bool connectionRequestHandler(void* parameter, const char* ipAddress)
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
    }
}

static void pumpReadData(IMasterConnection con);
int main(int argc, char** argv)
{
    (void)argc; (void)argv;
    signal(SIGINT, on_sigint);

    CS104_Slave slave = CS104_Slave_create(10, 10);
    g_alParams = CS104_Slave_getAppLayerParameters(slave);
    printf("[DBG] g_alParams=%p\n", g_alParams);
    if (!g_alParams) {
        printf("FATAL: g_alParams is NULL\n");
        return -1;
    }
    CS104_Slave_setLocalAddress(slave, "0.0.0.0");
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
    CS104_Slave_start(slave);
    if (!CS104_Slave_isRunning(slave)) {
        printf("Starting server failed!\n");
        CS104_Slave_destroy(slave);
        return -1;
    }
    printf("Server started, press Ctrl+C to stop.\n");

    while (g_running) {
        Thread_sleep(1000);

        if (g_needSendReadActAck) {
            g_needSendReadActAck = false;
            sendReadActAck(g_conn);
        }

        if (g_readTx.active) {
            pumpReadData(g_conn);
        }

        if (g_dirSending) {
            if (g_conn == NULL) {
                printf("[FILE] want send dirAck but g_conn=NULL\n");
            } else {
                printf("[FILE] sendDirAckPage... idx=%d/%d\n", g_dirIndex, g_dirCount);
                sendDirAckPage(g_conn);
            }
        }
    }


    printf("Stopping server\n");
    CS104_Slave_stop(slave);
    freeReadTx();
    CS104_Slave_destroy(slave);

    return 0;
}
