#ifndef __FPGA_RESET_GPIO_H__
#define __FPGA_RESET_GPIO_H__

#define SPI1_CS               8
#define FPGA_RESET_GPIO       42
#define FPGA_CDONE_GPIO       47
#define SPI8_SS               36
#define SPI10_CS              73
#define GPIO_CLOCK            52

#define DUMMY_BUF_SIZE        64

#define FPGA_IOCTL_MAGIC                  'a'

#define FPGA_IOCTL_PIO_SET_DIRCTION       _IOW(FPGA_IOCTL_MAGIC, 0, unsigned)
#define FPGA_IOCTL_PIO_READ               _IOR(FPGA_IOCTL_MAGIC, 1, unsigned)
#define FPGA_IOCTL_PIO_WRITE              _IOW(FPGA_IOCTL_MAGIC, 2, unsigned)
#define FPGA_IOCTL_SPI_READ               _IOW(FPGA_IOCTL_MAGIC, 3, unsigned)
#define FPGA_IOCTL_SPI_WRITE              _IOW(FPGA_IOCTL_MAGIC, 4, unsigned)

#define FPGA_PIO_INPUT        0
#define FPGA_PIO_OUTPUT       1

#define FPGA_CPU_PIO          0
#define FPGA_PM_PIO           1

#define FPGA_SPI_BUFF_SIZE    128


typedef struct
{
   int gpio;
   int pm_gpio;
   int id;
   char name[8];
} FPGA_PIO_t;

typedef struct
{
   int id;
   int val;
} FPGA_IOCTL_PIO_t;

typedef struct
{
   int spi;
   int size;
   unsigned char buff[FPGA_SPI_BUFF_SIZE];
} FPGA_IOCTL_SPI_t;

#define FPGA_BANK1_CPU_PIO_COUNT    20
static FPGA_PIO_t fpga_bank1_cpu_pio[FPGA_BANK1_CPU_PIO_COUNT] = {
   {  122,  0, 104, "pio1_04" },
   {   39,  0, 105, "pio1_05" },
   {  124,  0, 106, "pio1_06" },
   {   41,  0, 107, "pio1_07" },
   
   {   12,  0, 110, "pio1_10" },
   {   13,  0, 111, "pio1_11" },
   
   {  123,  0, 113, "pio1_13" },
   {   18,  0, 114, "pio1_14" },
   {   19,  0, 115, "pio1_15" },
   
   {   14,  0, 118, "pio1_18" },
   {   15,  0, 119, "pio1_19" },
   {   16,  0, 120, "pio1_20" },
   
   {   17,  0, 123, "pio1_23" },
   
   {   97,  0, 125, "pio1_25" },
   {   98,  0, 126, "pio1_26" },
   {  142,  0, 127, "pio1_27" },
   
   {   38,  0, 129, "pio1_29" },
   
   {   40,  0, 131, "pio1_31" },
   
   {  130,  0, 133, "pio1_33" },
   {  131,  0, 134, "pio1_34" },
};

#define FPGA_BANK2_CPU_PIO_COUNT    18    //27
static FPGA_PIO_t fpga_bank2_cpu_pio[FPGA_BANK2_CPU_PIO_COUNT] = {
   /* current using by GSBI1 */
   /*
   {    6,  0, 203, "pio2_03" },
   {    7,  0, 204, "pio2_04" },
   {    8,  0, 205, "pio2_05" },
   */
   {   53,  0, 207, "pio2_07" },
   /* currently using by GSBI10 */
   /*
   {   71,  0, 209, "pio2_09" },
   {   72,  0, 210, "pio2_10" },
   {   73,  0, 211, "pio2_11" },
   {   74,  0, 212, "pio2_12" },
   */
   /* current using by GSBI1 */
   //{    9,  0, 214, "pio2_14" },
   
   {   93,  0, 215, "pio2_15" },
   {   94,  0, 216, "pio2_16" },
   {   95,  0, 217, "pio2_17" },
   
   {   51,  0, 220, "pio2_20" },
   
   {   96,  0, 221, "pio2_21" },
   /* used by FPGA clock */
   //{   52,  0, 222, "pio2_22" },
   
   {   56,  0, 223, "pio2_23" },
   {   68,  0, 224, "pio2_24" },
   
   {   81,  0, 228, "pio2_28" },
   {  110,  0, 229, "pio2_29" },
   {   79,  0, 230, "pio2_30" },
   {   80,  0, 231, "pio2_31" },
   
   {   78,  0, 234, "pio2_34" },
   {  109,  0, 235, "pio2_35" },
   {  141,  0, 236, "pio2_36" },
   {  139,  0, 237, "pio2_37" },
   
   {   50,  0, 238, "pio2_38" },
   {   58,  0, 239, "pio2_39" },
};

#define FPGA_BANK1_PM_PIO_COUNT    5
static FPGA_PIO_t fpga_bank1_pm_pio[FPGA_BANK1_PM_PIO_COUNT] = {
   {   44,  0, 124, "pio1_24" },
    
   {   40,  0, 136, "pio1_36" },
   {   41,  0, 137, "pio1_37" },
   
   {   24,  0, 140, "pio1_40" },
   {   25,  0, 141, "pio1_41" },
};


#endif