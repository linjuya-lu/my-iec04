#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdint.h>
#include <errno.h>

#include "hal_thread.h"
#include "iec60870_common.h"
#include "cs104_connection.h"
#include "cs101_information_objects.h"

#ifndef FILE_IOA
#define FILE_IOA 0
#endif

#define OP_DIR_CALL        1  // 读目录
#define OP_DIR_CALL_CON    2  // 读目录确认
#define OP_READ_ACT        3  // 读文件激活
#define OP_READ_ACT_ACK    4  // 读文件激活确认
#define OP_READ_DATA       5  // 读文件数据
#define OP_READ_DATA_CON   6  // 读文件数据确认

static volatile bool g_running = true;
static volatile bool g_dir_done  = false;
static volatile bool g_read_done = false;


static volatile bool g_needFinish = false;
static uint32_t g_finishFileId = 0;
static uint32_t g_finishSegNo  = 0;
static uint32_t g_dirId = 1;
static uint32_t g_fileId = 0;
static uint32_t g_fileSize = 0;
static char     g_fileName[256] = {0};
static uint8_t* g_fileBuf = NULL;
static uint32_t g_fileReceived = 0;
static CS104_Connection g_con = NULL;
static CS101_AppLayerParameters g_alParams = NULL;

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

static void destroy_io(FileExt210 io)
{
    InformationObject_destroy((InformationObject)io);
}

static void rawMessageHandler(void* parameter, uint8_t* msg, int msgSize, bool sent)
{
    (void)parameter;
    printf("%s (%d): ", sent ? "SEND" : "RCVD", msgSize);
    for (int i = 0; i < msgSize; i++)
        printf("%02x ", msg[i]);
    printf("\n");
    fflush(stdout);
}

static bool asduReceivedHandler(void* parameter, int address, CS101_ASDU asdu)
{
    (void)parameter;
    (void)address;
    TypeID ti = CS101_ASDU_getTypeID(asdu);
    if ((int)ti != 210)
        return true;
    FileExt210 fe = (FileExt210)CS101_ASDU_getElement(asdu, 0);
    if (!fe)
        return true;
    uint8_t op = fe->op;
    if (op == OP_DIR_CALL_CON) {
        uint8_t result  = fe->u.dirCallAck.result;
        uint32_t dirId  = fe->u.dirCallAck.dirId;
        uint8_t hasMore = fe->u.dirCallAck.hasMore;
        uint8_t n       = fe->u.dirCallAck.fileCount;
        printf("DIR_CALL_CON result=%u dirId=%u fileCount=%u hasMore=%u\n",
               result, dirId, n, hasMore);
        for (uint8_t i = 0; i < n; i++) {
            const File210_DirFileEntry* e = &fe->u.dirCallAck.files[i];
            char name[128] = {0};
            uint8_t len = e->nameLen;
            if (len > sizeof(name) - 1) len = sizeof(name) - 1;
            memcpy(name, e->name, len);
            printf("  - %s  size=%u\n", name, (unsigned)e->size);
            if (g_fileName[0] == '\0' && i == 0) {
                strncpy(g_fileName, name, sizeof(g_fileName) - 1);
                g_fileName[sizeof(g_fileName) - 1] = '\0';
            }
        }
        if (!hasMore) {
            g_dir_done = true;
            printf("DIR done. selected file=%s\n", g_fileName);
        }
    }
    else if (op == OP_READ_ACT_ACK) {
        uint8_t  result   = fe->u.readActAck.result;
        uint32_t fileId   = fe->u.readActAck.fileId;
        uint32_t fileSize = fe->u.readActAck.fileSize;
        printf("[READ_ACT_ACK result=%u fileId=%u fileSize=%u\n",
               result, fileId, fileSize);
        if (result != 0 || fileSize == 0) {
            printf("READ_ACT failed\n");
            g_read_done = true;
            return true;
        }
        g_fileId = fileId;
        g_fileSize = fileSize;
        free(g_fileBuf);
        g_fileBuf = (uint8_t*)calloc(1, g_fileSize);
        g_fileReceived = 0;
        if (!g_fileBuf) {
            printf("[calloc fileBuf failed\n");
            g_read_done = true;
        }
    }
    else if (op == OP_READ_DATA) {
        uint32_t fileId  = fe->u.readData.fileId;
        uint32_t segNo   = fe->u.readData.segNo;     
        uint8_t  hasMore = fe->u.readData.hasMore;
        const uint8_t* data = fe->u.readData.data;
        int      len     = fe->u.readData.dataLen;
        uint8_t  cks     = fe->u.readData.checksum;
        if (fileId != g_fileId || !g_fileBuf || g_fileSize == 0) {
            printf("READ_DATA ignored (no active file)\n");
            return true;
        }

        if (len <= 0 || data == NULL) {
            printf("READ_DATA empty segNo=%u len=%d\n", segNo, len);
            return true;
        }

        if (segNo >= g_fileSize || segNo + (uint32_t)len > g_fileSize) {
            printf("READ_DATA overflow segNo=%u len=%d size=%u\n",
                   segNo, len, g_fileSize);
            g_read_done = true;
            return true;
        }
        uint8_t my = checksum8(data, (uint32_t)len);
        if (my != cks) {
            printf("READ_DATA checksum mismatch! segNo=%u len=%d got=%02x expect=%02x\n",
                   segNo, len, my, cks);
        }
        memcpy(g_fileBuf + segNo, data, (size_t)len);
        g_fileReceived += (uint32_t)len;
        printf("READ_DATA segNo=%u len=%d hasMore=%u progress≈%u/%u\n",
               segNo, len, hasMore, g_fileReceived, g_fileSize);
        if (!hasMore || (segNo + (uint32_t)len >= g_fileSize)) {
            printf("READ_DATA completed. defer ACK+save to main loop\n");
            g_finishFileId = fileId;
            g_finishSegNo  = segNo;
            g_needFinish   = true;   
        }
    }
    fflush(stdout);
    return true;
}

FileExt210
FileExt210_createDirCall(int           ioa,
                         uint32_t      dirId,
                         const uint8_t *name,
                         uint8_t       nameLen,
                         uint8_t       callFlag,
                         const uint8_t *beginTime,
                         const uint8_t *endTime);

static bool sendDirCall(const char* dirName)
{
    uint8_t nameLen = 0;
    const uint8_t* name = NULL;

    if (dirName && dirName[0]) {
        name = (const uint8_t*)dirName;
        nameLen = (uint8_t)strlen(dirName);
        if (nameLen > FILE210_MAX_NAME) nameLen = FILE210_MAX_NAME;
    }

    uint8_t callFlag = 0;
    const uint8_t* beginTime = NULL;
    const uint8_t* endTime   = NULL;

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_ACTIVATION, 0, 1, false, false);

    FileExt210 io = FileExt210_createDirCall(
        FILE_IOA,
        g_dirId,
        name,
        nameLen,
        callFlag,
        beginTime,
        endTime
    );
    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = CS104_Connection_sendASDU(g_con, a);
    CS101_ASDU_destroy(a);
    destroy_io(io);
    printf("SEND DIR_CALL dirId=%u nameLen=%u ok=%d\n", g_dirId, nameLen, ok);
    fflush(stdout);
    return ok;
}

static bool sendReadAct(const char* fileName)
{
    uint8_t nameLen = (uint8_t)strlen(fileName);
    if (nameLen > 200) nameLen = 200;

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_ACTIVATION, 0, 1, false, false);
    FileExt210 io = FileExt210_createReadAct(FILE_IOA, (const uint8_t*)fileName, nameLen);

    CS101_ASDU_addInformationObject(a, (InformationObject)io);

    bool ok = CS104_Connection_sendASDU(g_con, a);

    CS101_ASDU_destroy(a);
    destroy_io(io);
    printf("SEND READ_ACT file=%s ok=%d\n", fileName, ok);
    fflush(stdout);
    return ok;
}

static void doFinishAckAndSave(void)
{
    /*发送READ_DATA_ACK*/
    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);
    FileExt210 io = FileExt210_createReadDataAck(FILE_IOA, g_finishFileId, g_finishSegNo, 0);
    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = CS104_Connection_sendASDU(g_con, a);
    printf("SEND READ_DATA_ACK fileId=%u segNo=%u ok=%d\n", g_finishFileId, g_finishSegNo, ok);
    CS101_ASDU_destroy(a);
    destroy_io(io);
    /*落盘*/
    char out[512];
    snprintf(out, sizeof(out), "recv_%s", (g_fileName[0] ? g_fileName : "file.bin"));
    FILE* fp = fopen(out, "wb");
    if (fp) {
        size_t wr = fwrite(g_fileBuf, 1, g_fileSize, fp);
        fclose(fp);
        printf("saved to %s (%u bytes, wrote=%zu)\n", out, g_fileSize, wr);
    } else {
        printf("fopen(%s) failed: %s\n", out, strerror(errno));
    }
    fflush(stdout);
    g_read_done = true;
}

int main(int argc, char** argv)
{
    signal(SIGINT, on_sigint);
    const char* ip = (argc > 1) ? argv[1] : "172.16.19.193";
    int port = (argc > 2) ? atoi(argv[2]) : 2404;
    const char* dirName = (argc > 3) ? argv[3] : "";
    const char* wantFile = (argc > 4) ? argv[4] : NULL;

    if (wantFile) {
        strncpy(g_fileName, wantFile, sizeof(g_fileName) - 1);
        g_fileName[sizeof(g_fileName) - 1] = '\0';
    }

    g_con = CS104_Connection_create(ip, port);
    CS104_Connection_setRawMessageHandler(g_con, rawMessageHandler, NULL);
    CS104_Connection_setASDUReceivedHandler(g_con, asduReceivedHandler, NULL);

    if (!CS104_Connection_connect(g_con)) {
        printf("connect failed\n");
        CS104_Connection_destroy(g_con);
        return -1;
    }
    CS104_Connection_sendStartDT(g_con);
    g_alParams = CS104_Connection_getAppLayerParameters(g_con);
    printf("connected to %s:%d\n", ip, port);
    fflush(stdout);

    sendDirCall(dirName);

    while (g_running) {
        Thread_sleep(200);
        if (g_needFinish) {
            g_needFinish = false;
            doFinishAckAndSave();
            g_running = false;
            continue;
        }

        if (g_dir_done && !g_read_done) {
            static bool started = false;
            if (!started) {
                started = true;
                if (g_fileName[0] == '\0') {
                    printf("no file selected, stop\n");
                    g_running = false;
                } else {
                    sendReadAct(g_fileName);
                }
            }
        }
        if (g_read_done) {
            g_running = false;
        }
    }
    printf("closing...\n");
    fflush(stdout);
    CS104_Connection_sendStopDT(g_con);
    CS104_Connection_close(g_con);
    CS104_Connection_destroy(g_con);
    free(g_fileBuf);
    g_fileBuf = NULL;

    return 0;
}
