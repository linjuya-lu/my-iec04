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
#include <pthread.h>

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

typedef struct {
    bool     loaded;
    bool     active;
    uint32_t fileId;
    uint32_t fileSize;
    uint32_t offset;
    char     fileName[256];
    uint8_t* data;
} ReadTxCtx;

typedef struct ClientCtx {
    IMasterConnection con;

    // dir state
    bool     dirSending;
    uint32_t dirId;
    uint8_t  dirResult;
    time_t   dirMtime[MAX_DIR_FILES];
    char     dirFiles[MAX_DIR_FILES][128];
    uint32_t dirSizes[MAX_DIR_FILES];
    int      dirCount;
    int      dirIndex;

    // file state
    ReadTxCtx readTx;
    bool      needSendReadActAck;
    bool closing;
    struct ClientCtx* next;
} ClientCtx;

static ClientCtx* g_clients = NULL;
static pthread_mutex_t g_clients_mtx = PTHREAD_MUTEX_INITIALIZER;


static ClientCtx* findClient(IMasterConnection con)
{
    for (ClientCtx* it = g_clients; it; it = it->next)
        if (it->con == con) return it;
    return NULL;
}

static void freeReadTxCtx(ReadTxCtx* tx)
{
    if (tx->data) free(tx->data);
    memset(tx, 0, sizeof(*tx));
}

static ClientCtx* addClient(IMasterConnection con)
{
    ClientCtx* c = (ClientCtx*)calloc(1, sizeof(ClientCtx));
    c->con = con;
    c->next = g_clients;
    g_clients = c;
    return c;
}

static void removeClient(IMasterConnection con)
{
    ClientCtx** pp = &g_clients;
    while (*pp) {
        ClientCtx* cur = *pp;
        if (cur->con == con) {
            *pp = cur->next;
            freeReadTxCtx(&cur->readTx);
            free(cur);
            return;
        }
        pp = &cur->next;
    }
}


//全局
static volatile bool g_running = true;
static CS101_AppLayerParameters g_alParams = NULL;


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

static void buildDirList(ClientCtx* ctx, const char* dirPath)
{
    printf("[FILE] buildDirList path=%s\n", dirPath);

    ctx->dirCount = 0;
    ctx->dirIndex = 0;
    ctx->dirResult = 0;

    DIR* d = opendir(dirPath);
    if (!d) {
        printf("[FILE] opendir failed: %s (%s)\n", dirPath, strerror(errno));
        ctx->dirResult = 1;
        return;
    }

    struct dirent* ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        if (ctx->dirCount >= MAX_DIR_FILES) break;

        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dirPath, ent->d_name);

        struct stat st;
        if (stat(full, &st) != 0) continue;
        if (!S_ISREG(st.st_mode)) continue;

        strncpy(ctx->dirFiles[ctx->dirCount], ent->d_name, sizeof(ctx->dirFiles[0]) - 1);
        ctx->dirFiles[ctx->dirCount][sizeof(ctx->dirFiles[0]) - 1] = '\0';
        ctx->dirSizes[ctx->dirCount] = (uint32_t)st.st_size;
        ctx->dirMtime[ctx->dirCount] = st.st_mtime;
        ctx->dirCount++;
    }

    closedir(d);
    printf("[FILE] dir list ready: path=%s files=%d\n", dirPath, ctx->dirCount);
}

static void sendDirAckPage(ClientCtx* ctx)
{
    if (!ctx || !ctx->con) return;

    IMasterConnection con = ctx->con;

    /* ====== 即使没有文件也要回一帧 ACK ====== */
    if (ctx->dirCount <= 0) {
        File210_DirFileEntry entries[FILE210_MAX_DIR_FILES];
        memset(entries, 0, sizeof(entries));

        uint8_t n = 0;
        uint8_t hasMore = 0;

        CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);

        FileExt210 io = FileExt210_createDirCallAck(
            FILE_IOA,
            ctx->dirResult,  /* 0成功 1失败（buildDirList 里设） */
            ctx->dirId,
            hasMore,
            n,
            entries
        );

        CS101_ASDU_addInformationObject(a, (InformationObject)io);
        bool ok = IMasterConnection_sendASDU(con, a);

        printf("[FILE] SEND DIR_CALL_ACK(EMPTY) con=%p dirId=%u result=%u n=%u hasMore=%u ok=%d\n",
               con, ctx->dirId, ctx->dirResult, n, hasMore, ok);

        CS101_ASDU_destroy(a);
        InformationObject_destroy((InformationObject)io);

        ctx->dirSending = false;
        return;
    }

    /* ====== 分页发送 ====== */
    if (ctx->dirIndex >= ctx->dirCount) {
        ctx->dirSending = false;
        return;
    }

    File210_DirFileEntry entries[FILE210_MAX_DIR_FILES];
    memset(entries, 0, sizeof(entries));

    int left = ctx->dirCount - ctx->dirIndex;
    uint8_t n = (left > FILE210_MAX_DIR_FILES) ? FILE210_MAX_DIR_FILES : (uint8_t)left;
    uint8_t hasMore = (ctx->dirIndex + n < ctx->dirCount) ? 1 : 0;

    for (uint8_t i = 0; i < n; i++) {
        const char* name = ctx->dirFiles[ctx->dirIndex + i];
        uint8_t nameLen = (uint8_t)strlen(name);
        if (nameLen > FILE210_MAX_NAME) nameLen = FILE210_MAX_NAME;

        entries[i].nameLen = nameLen;
        memcpy(entries[i].name, name, nameLen);
        entries[i].attr = 0;
        entries[i].size = ctx->dirSizes[ctx->dirIndex + i];
        fill_cp56time2a_from_time_t(ctx->dirMtime[ctx->dirIndex + i], entries[i].time);
    }

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);

    FileExt210 io = FileExt210_createDirCallAck(
        FILE_IOA,
        ctx->dirResult,
        ctx->dirId,
        hasMore,
        n,
        entries
    );

    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(con, a);

    printf("[FILE] SEND DIR_CALL_ACK con=%p dirId=%u result=%u n=%u hasMore=%u ok=%d\n",
           con, ctx->dirId, ctx->dirResult, n, hasMore, ok);

    CS101_ASDU_destroy(a);
    InformationObject_destroy((InformationObject)io);

    ctx->dirIndex += n;
    if (!hasMore) ctx->dirSending = false;
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
static void sendReadActAck(ClientCtx* ctx)
{
    if (!ctx || !ctx->con || !g_alParams) return;

    ReadTxCtx* tx = &ctx->readTx;

    uint8_t result = (tx->data && tx->fileSize > 0) ? 0 : 1;

    /* 更安全：避免 fileName 没有 '\0' 导致 strlen 越界 */
    size_t sl = strnlen(tx->fileName, sizeof(tx->fileName));
    if (sl > 255) sl = 255;
    uint8_t nameLen = (uint8_t)sl;

    const uint8_t* name = (const uint8_t*)tx->fileName;

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_ACTIVATION_CON, 0, 1, false, false);
    if (!a) return;

    FileExt210 io = FileExt210_createReadActAck(
        FILE_IOA,
        result,
        name,
        nameLen,
        tx->fileId,
        tx->fileSize
    );

    if (!io) {
        CS101_ASDU_destroy(a);
        return;
    }

    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(ctx->con, a);

    printf("[FILE] SEND READ_ACT_ACK con=%p result=%u name=%s fileId=%u size=%u ok=%d\n",
           ctx->con, result, tx->fileName, tx->fileId, tx->fileSize, ok);

    CS101_ASDU_destroy(a);
    InformationObject_destroy((InformationObject)io);

    if (result == 0) {
        tx->offset = 0;
        tx->active = true;
    } else {
        tx->active = false;
    }
}


static void pumpReadData(ClientCtx* ctx)
{
    if (!ctx || !ctx->con || !g_alParams) return;

    ReadTxCtx* tx = &ctx->readTx;

    if (!tx->active) return;
    if (!tx->data || tx->fileSize == 0) { tx->active = false; return; }

    if (tx->offset >= tx->fileSize) {
        printf("[FILE] READ_DATA all sent, wait op=6 con=%p fileId=%u\n", ctx->con, tx->fileId);
        tx->active = false;
        return;
    }

    uint32_t left = tx->fileSize - tx->offset;
    uint32_t n = (left > FILE_CHUNK) ? FILE_CHUNK : left;

    uint32_t segNo = tx->offset;  /* 用偏移当段号 */
    uint8_t  hasMore = (tx->offset + n < tx->fileSize) ? 1 : 0;

    /* 注意：seg 的所有权通常会被 FileExt210_createReadData 接管，
       你原代码也没 free(seg)，这里保持一致 */
    uint8_t* seg = (uint8_t*)malloc(n);
    if (!seg) { printf("[FILE] malloc seg failed\n"); return; }
    memcpy(seg, tx->data + tx->offset, n);

    uint8_t cks = checksum8(seg, n);

    CS101_ASDU a = CS101_ASDU_create(g_alParams, false, CS101_COT_REQUEST, 0, 1, false, false);
    if (!a) { free(seg); return; }

    FileExt210 io = FileExt210_createReadData(FILE_IOA, tx->fileId, segNo, hasMore, seg, (int)n, cks);
    if (!io) { CS101_ASDU_destroy(a); free(seg); return; }

    CS101_ASDU_addInformationObject(a, (InformationObject)io);
    bool ok = IMasterConnection_sendASDU(ctx->con, a);

    printf("[FILE] SEND READ_DATA con=%p fileId=%u segNo=%u len=%u hasMore=%u cks=%02x ok=%d\n",
           ctx->con, tx->fileId, segNo, n, hasMore, cks, ok);

    CS101_ASDU_destroy(a);
    InformationObject_destroy((InformationObject)io);

    if (ok) tx->offset += n;
}

static bool asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    (void)parameter;

    TypeID ti = CS101_ASDU_getTypeID(asdu);
    if ((int)ti != 210) return true;

    pthread_mutex_lock(&g_clients_mtx);
    ClientCtx* ctx = findClient(connection);
    if (!ctx) ctx = addClient(connection);
    pthread_mutex_unlock(&g_clients_mtx);

    FileExt210 fe = (FileExt210)CS101_ASDU_getElement(asdu, 0);
    if (!fe) return true;

    uint8_t op = fe->op;

    if (op == OP_DIR_CALL) {
        ctx->dirId = fe->u.dirCall.dirId;
        buildDirList(ctx, FILE_DIR_BASE);
        ctx->dirSending = true;     // 只置位，不直接发送也行（让主循环发）
        // 你也可以这里直接 sendDirAckPage(ctx) 先回一页
    }
    else if (op == OP_READ_ACT) {
        freeReadTxCtx(&ctx->readTx);

        uint8_t nameLen = fe->u.readAct.nameLen;
        if (nameLen > sizeof(ctx->readTx.fileName) - 1) nameLen = sizeof(ctx->readTx.fileName) - 1;
        memcpy(ctx->readTx.fileName, fe->u.readAct.name, nameLen);
        ctx->readTx.fileName[nameLen] = '\0';

        char path[512];
        snprintf(path, sizeof(path), "%s/%s", FILE_DIR_BASE, ctx->readTx.fileName);

        FILE* fp = fopen(path, "rb");
        if (!fp) {
            printf("[FILE] open failed: %s (%s)\n", path, strerror(errno));
            ctx->readTx.loaded = false;
        } else {
            fseek(fp, 0, SEEK_END);
            long sz = ftell(fp);
            fseek(fp, 0, SEEK_SET);

            if (sz > 0) {
                ctx->readTx.data = (uint8_t*)malloc((size_t)sz);
                if (ctx->readTx.data) {
                    size_t rd = fread(ctx->readTx.data, 1, (size_t)sz, fp);
                    ctx->readTx.fileSize = (uint32_t)rd;
                    ctx->readTx.fileId = 1;
                    ctx->readTx.loaded = true;
                }
            }
            fclose(fp);
        }

        ctx->needSendReadActAck = true;   // 让主循环发 ACK
    }
    else if (op == OP_READ_DATA_CON) {
        freeReadTxCtx(&ctx->readTx);
    }

    return true;
}


static bool connectionRequestHandler(void* parameter, const char* ipAddress)
{
    (void)parameter;
    printf("New connection request from %s\n", ipAddress);
    return true;
}

static void connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    (void)parameter;

    switch (event) {

    case CS104_CON_EVENT_CONNECTION_OPENED:
        pthread_mutex_lock(&g_clients_mtx);
        if (!findClient(con)) addClient(con);
        pthread_mutex_unlock(&g_clients_mtx);
        printf("Connection opened (%p)\n", con);
        break;

    case CS104_CON_EVENT_CONNECTION_CLOSED:
        pthread_mutex_lock(&g_clients_mtx);
        ClientCtx* c = findClient(con);
        if (c) c->closing = true;
        pthread_mutex_unlock(&g_clients_mtx);
        printf("Connection closed (%p)\n", con);
        break;


    case CS104_CON_EVENT_ACTIVATED:
        /* 变成 active：可以开始给它发送分页/文件数据 */
        printf("Connection activated (%p)\n", con);
        break;

    case CS104_CON_EVENT_DEACTIVATED:
        /* 变成 standby：建议不要 remove ctx（连接还在），
           但你可以停止该连接的发送行为，避免往 standby 发数据 */
        printf("Connection deactivated (%p)\n", con);
        break;

    default:
        break;
    }
}

static void sweepClosingClients(void)
{
    ClientCtx** pp = &g_clients;
    while (*pp) {
        ClientCtx* cur = *pp;
        if (cur->closing) {
            *pp = cur->next;
            freeReadTxCtx(&cur->readTx);
            free(cur);
            continue;
        }
        pp = &cur->next;
    }
}



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
    CS104_Slave_setMaxOpenConnections(slave, 8);

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

    /* 临时快照数组：保存本轮要处理的客户端指针 */
    ClientCtx* snap[64]; /* 够用就行，不够就截断；可按需调大 */
    while (g_running) {
        Thread_sleep(50);

        int n = 0;

        /* 1) 锁内：只做快照 + 清标志（不做任何 send） */
        pthread_mutex_lock(&g_clients_mtx);
        for (ClientCtx* c = g_clients; c; c = c->next) {
            if (n >= (int)(sizeof(snap) / sizeof(snap[0]))) break;
            snap[n++] = c;
        }

        /* 把本轮要发送的“触发位”先拷出来，锁内只清标志 */
        bool doReadAck[64];
        bool doPump[64];
        bool doDir[64];

        for (int i = 0; i < n; ++i) {
            ClientCtx* c = snap[i];

            doReadAck[i] = c->needSendReadActAck;
            if (c->needSendReadActAck) c->needSendReadActAck = false;

            doPump[i] = c->readTx.active;
            doDir[i]  = c->dirSending;
        }
        pthread_mutex_unlock(&g_clients_mtx);

        /* 2) 锁外：真正执行发送/处理（避免占锁时阻塞） */
        for (int i = 0; i < n; ++i) {
            ClientCtx* c = snap[i];

            if (doReadAck[i]) {
                sendReadActAck(c);
            }

            if (doPump[i]) {
                pumpReadData(c);
            }

            if (doDir[i]) {
                sendDirAckPage(c);
            }
        }
    }

    printf("Stopping server\n");
    CS104_Slave_stop(slave);

    /* 退出清理：把链表摘下来，锁外释放 */
    pthread_mutex_lock(&g_clients_mtx);
    ClientCtx* it = g_clients;
    g_clients = NULL;
    pthread_mutex_unlock(&g_clients_mtx);

    while (it) {
        ClientCtx* next = it->next;
        freeReadTxCtx(&it->readTx);
        free(it);
        it = next;
    }

    CS104_Slave_destroy(slave);
    return 0;
}

