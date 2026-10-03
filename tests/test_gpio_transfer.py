from pathlib import Path
s=(Path(__file__).resolve().parents[1] / 'src/paw3222.c').read_text()
a=s.index('static int paw32xx_gpio_transfer(');b=s.index('\n#endif',a)
f=s[a:b]
pre=r'''
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#define GPIO_OUTPUT_INACTIVE 0
#define NRF_GPIO_PIN_NOPULL 0
#define PAW32XX_NRF_PSEL_CONNECT_BIT (1u<<31)
#define SPI_WRITE 128
struct gpio {int unused;};
struct paw32xx_config {struct {struct {struct {struct gpio gpio;} cs;} config;} spi;};
struct hw {struct {uint32_t SCK;} PSEL;};
struct paw32xx_data {bool spim_mosi_psel_saved,spim_miso_psel_saved;uint32_t spim_mosi_psel,spim_miso_psel,spim_sclk_psel;struct hw *spim;};
struct device {struct paw32xx_config *config;struct paw32xx_data *data;};
static int samples,cs,unstable,fail_cs;
static uint8_t response;
static uint32_t paw32xx_nrf52_psel_to_pin(uint32_t x){return x;}
static void paw32xx_nrf52_spim_deactivate(struct paw32xx_data*d){}
static void paw32xx_sdio_disconnect(struct paw32xx_data*d){}
static void nrf_gpio_pin_set(uint32_t p){}
static void nrf_gpio_pin_clear(uint32_t p){}
static void nrf_gpio_cfg_output(uint32_t p){}
static void nrf_gpio_cfg_input(uint32_t p,int mode){}
static void nrf_gpio_pin_write(uint32_t p,int v){}
static void k_busy_wait(int us){}
static int gpio_pin_configure_dt(const struct gpio*g,int v){return fail_cs?-EIO:0;}
static int paw32xx_force_cs(const struct device*d,bool low){cs=low;return 0;}
static uint32_t nrf_gpio_pin_read(uint32_t p){int n=samples++;uint32_t bit=(response>>(7-n/3))&1;return unstable && n==1 ? !bit:bit;}
'''
post=r'''
int main(void){struct paw32xx_config c={0};struct hw hw={0};struct paw32xx_data d={true,true,15,15,13,&hw};struct device dev={&c,&d};
for(int byte=0;byte<256;byte++){uint8_t v=0;samples=0;response=byte;unstable=0;assert(paw32xx_gpio_transfer(&dev,3,&v,false)==0);assert(v==byte);assert(samples==24);assert(cs==0);}
uint8_t v=0;samples=0;response=0x30;unstable=1;assert(paw32xx_gpio_transfer(&dev,3,&v,false)==-EIO);assert(cs==0);
samples=0;unstable=0;v=0x5a;assert(paw32xx_gpio_transfer(&dev,9,&v,true)==0);assert(samples==0);assert(cs==0);
d.spim_miso_psel=14;assert(paw32xx_gpio_transfer(&dev,3,&v,false)==-EINVAL);
puts("PASS: all 256 bytes, unstable-bit rejection, CS release, writes, invalid pin mapping");}
'''
import subprocess
import tempfile
with tempfile.TemporaryDirectory(prefix='paw3222-test-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(pre+f+post)
    subprocess.run(['cc', '-std=c11', '-Werror', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
