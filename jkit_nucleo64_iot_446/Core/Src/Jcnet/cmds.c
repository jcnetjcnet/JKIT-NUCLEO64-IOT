/*
 * cmds.c
 *
 *  Created on: 2022. 1. 31.
 *      Author: isjeon
 */

#include "main.h"
#include "stdio.h"

extern void led_fn(int ac, char *av[]);
extern void wiz_fn(int ac, char *av[]);
extern void rs485_fn(int ac, char *av[]);
extern void switch_fn(int ac, char *av[]);
extern void can_fn(int ac, char *av[]);
extern void esp32_fn(int ac, char *av[]);
extern void esp32_atlib_fn(int ac, char *av[]);
typedef struct _cmd_node_tag {
        const char *cmd;
        void (*fn)(int ac, char *av[]);
        const char *help;
} cmd_node_t;
const cmd_node_t cmd_tbl[] =
{
                {"led",     led_fn,    "Test Leds"},
                {"switch",  switch_fn, "Swith test"},
				{"wiz",     wiz_fn,    "wiznet function"},
				{"rs485",   rs485_fn,  "RS485 function"},
				{"can",     can_fn,    "CAN functions"},
				{"at",      esp32_fn,  "ESP32 at command"},
				{"esp32",   esp32_atlib_fn, "ESP32_AT_Lib"}
};

#include "string.h"

int exec_cmd(uint8_t *cmd, int ac,char *av[])
{
        int i;
        for( i = 0 ; i < sizeof(cmd_tbl) / sizeof(cmd_tbl[0]) ; i ++)
        {
                if(!strcmp(cmd, cmd_tbl[i].cmd))
                {
                        cmd_tbl[i].fn(ac, av);
                        return 0;
                }
        }
        return -1;
}
extern void my_putchar(char c);
int get_args(char *buf, char *av[])
{
        int     num, start, end;
        start = end = num = 0;
        while (1)
        {
//printf("buf+start = [%s] start=%d end=%d buf[end]=%x num=%d\n",buf, start,end,buf[end],num);
                if(buf[end] == '\0' || buf[end] == '\n' || buf[end] == '\r')
                {
                        if(buf[end]) buf[end] = '\0';
                        if(start != end)
                        {
                                strcpy(av[num],buf+start);
                                num ++;
                                return num;
                        }
                        else
                        {
                                return num;
                        }
                }
                if(buf[end] != ' ' && buf[end] != '\t' ) {
                        end ++;
                }
                else
                {
                        buf[end] = 0;
                        strcpy(av[num],buf+start);
                        num ++;
                        end ++;
                        start = end ;
                }
        }
        return 0;
}

char avbuf[6][64];
char *av[6] = {
                &avbuf[0][0],
                &avbuf[1][0],
                &avbuf[2][0],
                &avbuf[3][0],
                &avbuf[4][0],
                &avbuf[5][0]
};
const char *prompt="jkit";
char *version="iot2.1";
static char cmd_buf[128],old_buf[128];
static int idx = 0;
int do_cmd(char ch)
{

        char buf[128];
        int ac,i;
        if(ch == '\n' || ch == '\r')
        {
                 my_putchar('\n');
                 cmd_buf[idx] = '\0';
#if 1
                 if(!strncmp(cmd_buf,"!!",2))
                 {
                         strcpy(cmd_buf,old_buf);
                 }
#endif
                 strcpy(buf,cmd_buf);
                 for( i = 0 ; i < 6 ; i ++) av[i] = &avbuf[i][0];
                 ac = get_args(cmd_buf, av);
                 if(idx == 0 || !ac) {
                         idx = 0;
                         printf("%s-%s> ",prompt,version); fflush(stdout);
                         return 0;
                 }
                 strcpy(old_buf,buf);
                 exec_cmd(av[0],ac, av);
                 printf("%s-%s> ",prompt,version); fflush(stdout);
                 idx = 0;
        }
        else if(ch == '\b')
        {
                 if(idx > 0) { idx --; my_putchar('\b'); my_putchar(' '); my_putchar('\b'); return 0;}
        }
        else if(idx < 63) {
                 cmd_buf[idx++] = ch;   my_putchar(ch);
        }

        return 0;

}
