//#include "UsartWrite.hpp"
//#include <sys/unistd.h>
//
//USART_TypeDef* g_usart_write_instance = nullptr;
//
//void usart_write_bind(USART_TypeDef* instance)
//{
//    g_usart_write_instance = instance;
//}
//
//extern "C" int _write(int file, char* ptr, int len)
//{
//    USART_TypeDef* U = g_usart_write_instance;
//    if (!U) return len;
//
//    for (int i = 0; i < len; i++) {
//        while (!LL_USART_IsActiveFlag_TXE(U));
//        LL_USART_TransmitData8(U, ptr[i]);
//    }
//    while (!LL_USART_IsActiveFlag_TC(U));
//
//    return len;
//}
