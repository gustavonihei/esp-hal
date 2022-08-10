use crate::{
    clock::XtalClock, pac::EXTMEM, pac::I2S, pac::RTC_CNTL, pac::SPI0, pac::SPI1, pac::SYSTEM,
};

use crate::rtc_cntl::{RtcCalSel, RtcClock, RtcFastClock, RtcSlowClock};

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT: u16 = 0x10;

pub(crate) fn init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };
    rtc_cntl.ana_conf.modify(|_, w| w.pvtmon_pu().clear_bit());

    unsafe {
        rtc_cntl
            .timer1
            .modify(|_, w| w.pll_buf_wait().bits(20u8).ck8m_wait().bits(20u8));
        rtc_cntl.timer5.modify(|_, w| w.min_slp_val().bits(2u8));

        // Set default powerup & wait time
        rtc_cntl.timer3.modify(|_, w| {
            w.wifi_powerup_timer()
                .bits(1u8)
                .wifi_wait_timer()
                .bits(1u16)
        });
        rtc_cntl.timer4.modify(|_, w| {
            w.powerup_timer()
                .bits(1u8)
                .wait_timer()
                .bits(1u16)
                .dg_wrap_powerup_timer()
                .bits(1u8)
                .dg_wrap_wait_timer()
                .bits(1u16)
        });
        rtc_cntl.timer5.modify(|_, w| {
            w.rtcmem_powerup_timer()
                .bits(1u8)
                .rtcmem_wait_timer()
                .bits(1u16)
        });
        rtc_cntl.bias_conf.modify(|_, w| {
            w.dec_heartbeat_width()
                .set_bit()
                .dec_heartbeat_period()
                .set_bit()
        });
        rtc_cntl.reg.modify(|_, w| {
            w.dbias_wak()
                .bits(RTC_CNTL_DBIAS_1V10)
                .dbias_slp()
                .bits(RTC_CNTL_DBIAS_1V10)
        });
        rtc_cntl.timer2.modify(|_, w| {
            w.ulpcp_touch_start_wait()
                .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
        });
    }

    clock_control_init();

    power_control_init();

    calibrate_ocode();

    unsafe {
        rtc_cntl.int_ena_rtc.write(|w| w.bits(0));
        rtc_cntl.int_clr_rtc.write(|w| w.bits(u32::MAX));
    }
}

pub(crate) fn configure_clock() {
    assert!(matches!(
        RtcClock::get_xtal_freq(),
        XtalClock::RtcXtalFreq40M
    ));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClock8m);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRtc);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let rtc_cntl = &*RTC_CNTL::ptr();
        rtc_cntl.store1.write(|w| w.bits(cal_val));
    }
}

fn calibrate_ocode() {}

/// Perform clock control related initialization
fn clock_control_init() {
    let extmem = unsafe { &*EXTMEM::ptr() };
    let system = unsafe { &*SYSTEM::ptr() };
    let spi_mem_0 = unsafe { &*SPI0::ptr() };
    let spi_mem_1 = unsafe { &*SPI1::ptr() };

    // Clear CMMU clock force on
    extmem
        .pro_cache_mmu_power_ctrl
        .modify(|_, w| w.pro_cache_mmu_mem_force_on().clear_bit());

    // Clear ROM clock force on
    system
        .rom_ctrl_0
        .modify(|_, w| unsafe { w.rom_fo().bits(0) });

    // Clear SRAM clock force on
    system
        .sram_ctrl_0
        .modify(|_, w| unsafe { w.sram_fo().bits(0) });

    // Clear tag clock force on
    extmem
        .pro_dcache_tag_power_ctrl
        .modify(|_, w| w.pro_dcache_tag_mem_force_on().clear_bit());
    extmem
        .pro_icache_tag_power_ctrl
        .modify(|_, w| w.pro_icache_tag_mem_force_on().clear_bit());

    // Clear register clock force on
    spi_mem_0.w17.modify(|_, w| unsafe { w.bits(0) });
    spi_mem_1.w17.modify(|_, w| unsafe { w.bits(0) });
}

/// Perform power control related initialization
fn power_control_init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };
    let system = unsafe { &*SYSTEM::ptr() };
    rtc_cntl
        .clk_conf
        .modify(|_, w| w.ck8m_force_pu().clear_bit());

    // Cancel XTAL force PU if no need to force power up
    // Cannot cancel XTAL force PU if PLL is force power on
    rtc_cntl
        .options0
        .modify(|_, w| w.xtl_force_pu().clear_bit());

    // Clear APLL close
    rtc_cntl
        .ana_conf
        .modify(|_, w| w.plla_force_pu().clear_bit().plla_force_pd().set_bit());

    // Cancel BBPLL force PU if setting no force power up
    rtc_cntl.options0.modify(|_, w| {
        w.bbpll_force_pu()
            .clear_bit()
            .bbpll_i2c_force_pu()
            .clear_bit()
            .bb_i2c_force_pu()
            .clear_bit()
    });

    // Cancel RTC REG force PU
    rtc_cntl.pwc.modify(|_, w| {
        w.force_pu()
            .clear_bit()
            .fastmem_force_pu()
            .clear_bit()
            .slowmem_force_pu()
            .clear_bit()
            .fastmem_force_noiso()
            .clear_bit()
            .slowmem_force_noiso()
            .clear_bit()
    });
    rtc_cntl.reg.modify(|_, w| {
        w.regulator_force_pu()
            .clear_bit()
            .dboost_force_pu()
            .clear_bit()
            .dboost_force_pd()
            .set_bit()
    });

    rtc_cntl
        .ana_conf
        .modify(|_, w| w.sar_i2c_force_pd().clear_bit());

    // If this mask is enabled, all soc memories cannot enter power down mode.
    // We should control soc memory power down mode from RTC,
    // so we will not touch this register any more.
    system
        .mem_pd_mask
        .modify(|_, w| w.lslp_mem_pd_mask().clear_bit());

    rtc_sleep_pd();

    rtc_cntl
        .dig_pwc
        .modify(|_, w| w.dg_wrap_force_pu().clear_bit().wifi_force_pu().clear_bit());
    rtc_cntl.dig_iso.modify(|_, w| {
        w.dg_wrap_force_noiso()
            .clear_bit()
            .wifi_force_noiso()
            .clear_bit()
    });
    rtc_cntl.pwc.modify(|_, w| w.force_noiso().clear_bit());

    // Cancel digital PADS force no iso
    system
        .cpu_per_conf
        .modify(|_, w| w.cpu_wait_mode_force_on().clear_bit());

    // If SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0,
    // the CPU clock will be closed when CPU enter WAITI mode.
    rtc_cntl.dig_iso.modify(|_, w| {
        w.dg_pad_force_unhold()
            .clear_bit()
            .dg_pad_force_noiso()
            .clear_bit()
    });
}

/// Configure whether certain peripherals are powered down in deep sleep
fn rtc_sleep_pd() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };
    let i2s = unsafe { &*I2S::ptr() };

    rtc_cntl
        .dig_pwc
        .modify(|_, w| w.lslp_mem_force_pu().clear_bit());

    rtc_cntl.pwc.modify(|_, w| {
        w.fastmem_force_lpu()
            .clear_bit()
            .slowmem_force_lpu()
            .clear_bit()
    });

    i2s.pd_conf
        .modify(|_, w| w.plc_mem_force_pu().clear_bit().fifo_force_pu().clear_bit());
}
