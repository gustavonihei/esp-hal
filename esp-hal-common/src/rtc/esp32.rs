use crate::{clock::XtalClock, pac::DPORT, pac::RTC_CNTL};

use crate::rtc_cntl::{RtcCalSel, RtcClock, RtcFastClock, RtcSlowClock};

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_DBG_ATTEN_DEFAULT: u8 = 3;

pub(crate) fn init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };

    rtc_cntl.ana_conf.modify(|_, w| {
        w.pvtmon_pu()
            .clear_bit()
            .txrf_i2c_pu()
            .clear_bit()
            .rfrx_pbus_pu()
            .clear_bit()
            .ckgen_i2c_pu()
            .clear_bit()
            .pll_i2c_pu()
            .clear_bit()
    });

    rtc_cntl.timer1.modify(|_, w| unsafe {
        w.pll_buf_wait()
            .bits(20u8)
            .xtl_buf_wait()
            .bits(100u16)
            .ck8m_wait()
            .bits(20u8)
    });

    rtc_cntl.bias_conf.modify(|_, w| unsafe {
        w.dbg_atten()
            .bits(RTC_CNTL_DBG_ATTEN_DEFAULT)
            .dec_heartbeat_period()
            .set_bit()
            .dec_heartbeat_width()
            .set_bit()
    });

    // Reset RTC bias to default value (needed if waking up from deep sleep)
    rtc_cntl.reg.modify(|_, w| unsafe {
        w.dbias_wak()
            .bits(RTC_CNTL_DBIAS_1V10)
            .dbias_slp()
            .bits(RTC_CNTL_DBIAS_1V10)
    });

    clock_control_init();

    power_control_init();

    // Force power down wifi and bt power domain
    rtc_cntl.dig_iso.modify(|_, w| w.wifi_force_iso().set_bit());
    rtc_cntl.dig_pwc.modify(|_, w| w.wifi_force_pd().set_bit());

    unsafe {
        rtc_cntl.int_ena.write(|w| w.bits(0));
        rtc_cntl.int_clr.write(|w| w.bits(u32::MAX));
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

/// Perform clock control related initialization
fn clock_control_init() {
    let dport = unsafe { &*DPORT::ptr() };

    // Clear CMMU clock force on
    dport
        .pro_cache_ctrl1
        .modify(|_, w| w.pro_cmmu_force_on().clear_bit());
    dport
        .app_cache_ctrl1
        .modify(|_, w| w.app_cmmu_force_on().clear_bit());

    // Clear ROM clock force on
    dport.rom_fo_ctrl.modify(|_, w| unsafe {
        w.share_rom_fo()
            .bits(0)
            .app_rom_fo()
            .clear_bit()
            .pro_rom_fo()
            .clear_bit()
    });

    // Clear SRAM clock force on
    dport
        .sram_fo_ctrl_0
        .modify(|_, w| unsafe { w.sram_fo_0().bits(0) });
    dport
        .sram_fo_ctrl_1
        .modify(|_, w| w.sram_fo_1().clear_bit());

    // Clear tag clock force on
    dport.tag_fo_ctrl.modify(|_, w| {
        w.app_cache_tag_force_on()
            .clear_bit()
            .pro_cache_tag_force_on()
            .clear_bit()
    });
}

/// Perform power control related initialization
fn power_control_init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };

    rtc_cntl
        .clk_conf
        .modify(|_, w| w.ck8m_force_pu().clear_bit());

    rtc_cntl.options0.modify(|_, w| {
        // Cancel XTAL force PU
        w.xtl_force_pu()
            .clear_bit()
            // Cancel BIAS force PU
            .bias_core_force_pu()
            .clear_bit()
            .bias_i2c_force_pu()
            .clear_bit()
            .bias_force_nosleep()
            .clear_bit()
            // BIAS follow 8M
            .bias_core_folw_8m()
            .set_bit()
            .bias_i2c_folw_8m()
            .set_bit()
            .bias_sleep_folw_8m()
            .set_bit()
    });

    // Clear APLL close
    rtc_cntl
        .ana_conf
        .modify(|_, w| w.plla_force_pu().clear_bit().plla_force_pd().set_bit());
    rtc_cntl.options0.modify(|_, w| {
        w.bbpll_force_pu()
            .clear_bit()
            .bbpll_i2c_force_pu()
            .clear_bit()
    });

    // Cancel RTC REG force PU
    rtc_cntl.reg.modify(|_, w| {
        w.force_pu()
            .clear_bit()
            .dboost_force_pu()
            .clear_bit()
            .dboost_force_pd()
            .set_bit()
    });

    // Cancel digital force PU
    rtc_cntl.dig_pwc.modify(|_, w| {
        w.lslp_mem_force_pu()
            .clear_bit()
            .dg_wrap_force_pu()
            .clear_bit()
            .wifi_force_pu()
            .clear_bit()
            .rom0_force_pu()
            .clear_bit()
            .inter_ram0_force_pu()
            .clear_bit()
            .inter_ram1_force_pu()
            .clear_bit()
            .inter_ram2_force_pu()
            .clear_bit()
            .inter_ram3_force_pu()
            .clear_bit()
            .inter_ram4_force_pu()
            .clear_bit()
    });

    rtc_cntl.pwc.modify(|_, w| {
        w.slowmem_force_pu()
            .clear_bit()
            .fastmem_force_pu()
            .clear_bit()
            .force_pu()
            .clear_bit()
            .slowmem_force_noiso()
            .clear_bit()
            .fastmem_force_noiso()
            .clear_bit()
            .force_noiso()
            .clear_bit()
    });
    rtc_cntl.dig_iso.modify(|_, w| {
        w.dg_wrap_force_noiso()
            .clear_bit()
            .wifi_force_noiso()
            .clear_bit()
            .rom0_force_noiso()
            .clear_bit()
            .inter_ram0_force_noiso()
            .clear_bit()
            .inter_ram1_force_noiso()
            .clear_bit()
            .inter_ram2_force_noiso()
            .clear_bit()
            .inter_ram3_force_noiso()
            .clear_bit()
            .inter_ram4_force_noiso()
            .clear_bit()
            // Cancel digital PADS force no iso
            .dg_pad_force_unhold()
            .clear_bit()
            .dg_pad_force_noiso()
            .clear_bit()
    });
}
