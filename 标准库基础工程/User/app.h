#ifndef __APP_H
#define __APP_H

/* 应用层：任务创建与共享状态（实现在 app.c）。
 * main 完成外设初始化后调用 APP_Init()，再由 main 启动调度器。 */
void APP_Init(void);

#endif /* __APP_H */
