/*----------------------------------------------------------------------------/
/  FatFs - Generic FAT file system module  R0.12c 配置（本项目定制版）
/-----------------------------------------------------------------------------/
/ 基于官方 ffconf_template.h 裁剪，决策说明：
/  - _USE_LFN 0 + _CODE_PAGE 1：文件名全用 8.3 短名（如 20261004.CSV），
/    省掉整个 unicode 转换表和 LFN 工作缓冲（RAM/Flash 双省）
/  - _FS_TINY 1：FIL 对象不带私有扇区缓冲（省 512B RAM），本项目吞吐极低无影响
/  - _FS_LOCK 0 + _FS_REENTRANT 0：FatFS 只被 STG 任务访问（单写者），
/    无需文件锁与重入保护——若未来多任务访问必须改这两项
/  - _USE_MKFS 1：保留 f_mkfs 做兜底（卡无 FAT 文件系统时可板端格式化），
/    正常路径用 PC 格式化的 FAT32 卡
/  - _FS_NORTC 0：文件时间戳来自 get_fattime()（diskio_sd.c，RTC 无效时回 2026-01-01）
/----------------------------------------------------------------------------*/

#define _FFCONF 68300	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0
/* 0=读写（f_write/f_sync/f_unlink/f_getfree 可用） */

#define _FS_MINIMIZE	0
/* 0=全部基础函数可用（需要 f_getfree 看容量、f_unlink 清测试文件） */

#define	_USE_STRFUNC	0

#define _USE_FIND		0

#define	_USE_MKFS		0
/* 板端格式化已禁用（2026-10-07 实测）：8GB 卡 f_mkfs 需数分钟且中途无法喂
 * 看门狗/让出 CPU → 复位循环。卡统一在 PC 上格 FAT32；若未来要用 f_mkfs，
 * 必须挪到可喂心跳的专用流程（如分阶段状态机），严禁放在初始化路径 */

#define	_USE_FASTSEEK	0

#define	_USE_EXPAND		0

#define _USE_CHMOD		0

#define _USE_LABEL		0

#define	_USE_FORWARD	0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	1
/* 1 = 纯 ASCII（非 LFN 配置专用）：文件名只用数字/字母/下划线，无码表开销 */

#define	_USE_LFN	0
/* 0 = 关闭长文件名：日期文件名 YYYYMMDD.CSV 本身就是合法 8.3 短名 */

#define	_LFN_UNICODE	0

#define _STRF_ENCODE	3

#define _FS_RPATH	0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define _VOLUMES	1
/* 只有 SD 卡一个卷 */

#define _STR_VOLUME_ID	0
#define _VOLUME_STRS	"SD"

#define	_MULTI_PARTITION	0

#define	_MIN_SS		512
#define	_MAX_SS		512
/* SD 卡固定 512B 扇区（sdcard 驱动按 SDHC 规范实现） */

#define	_USE_TRIM	0

#define _FS_NOFSINFO	0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define	_FS_TINY	1
/* FIL 对象省掉 512B 私有缓冲，共用 FATFS 的公共缓冲——吞吐低的应用首选 */

#define _FS_EXFAT	0
/* 不支持 exFAT：卡必须在 PC 上格成 FAT32（板端 f_mkfs 兜底也是格 FAT32） */

#define _FS_NORTC	0
/* 0 = 使用 get_fattime()（diskio_sd.c 提供，RTC 无效时返回固定 2026-01-01） */
#define _NORTC_MON	1
#define _NORTC_MDAY	1
#define _NORTC_YEAR	2026

#define	_FS_LOCK	0
/* 单任务（STG）独占访问，无需文件锁 */

#define _FS_REENTRANT	0
/* 无需重入保护；若未来多任务访问同卷，改 1 并补 ff_cre_syncobj 等四钩子 */

/*--- End of configuration options ---*/
