#ifndef DEBUG_H
#define DEBUG_H

#include <stddef.h>

// USART2(PA2, ST-LINK VCP) 115200 8N1 へのノンブロッキング出力
void debug_init(void);
void debug_write(const char *buf, size_t len);   // リングバッファに積むだけ。満杯なら捨てる

#endif //DEBUG_H
