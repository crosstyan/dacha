use mocap_proto::mocap::{CameraHardwareConfig, ComputeModuleModel, SocBoard};


pub struct CameraHardwareConfigContainer {
    config: CameraHardwareConfig
}

impl_deref!(CameraHardwareConfigContainer::config as CameraHardwareConfig);

impl CameraHardwareConfigContainer {
    pub fn new(config: CameraHardwareConfig) -> Self {        
        Self {
            config
        }
    }

    pub fn is_rv1126b(&self) -> bool {
        self.config.soc_board() == SocBoard::ROCKCHIP_RV1126B
    }

    /// PWM (controller platform device, channel) that drives the sensor's
    /// trigger input and is looped back into the PTP clock's timestamp input
    /// (EXTTS channel 0). None: the PPS divider MCU makes the trigger.
    pub fn trigger_pwm(&self) -> Option<(&'static str, usize)> {
        if self.is_rv1126b() {
            // PWM1_CH2_M2 on GPIO6_A2 (CON3 pin 4) to the OV9281 FSIN and to
            // ETH_PPSTRIG_M0 on GPIO6_A0.
            return Some(("20720000.pwm", 0));
        }

        None
    }

    /// Whether the PTP clock's first pin is wired out to drive the MCU with a
    /// 1 Hz pulse.
    pub fn has_ptp_pps_output(&self) -> bool {
        !self.is_rv1126b()
    }

    pub fn has_accelerometer(&self) -> bool {
        !self.is_rv1126b()
    }

    /// The camera sensor to use when the board has more than one, as a
    /// substring of its media entity name. None picks the first sensor found.
    pub fn camera_sensor_name(&self) -> Option<&'static str> {
        if self.is_rv1126b() {
            // The colour AR0234 on the same adapter is not used for blobs.
            return Some("ov9281");
        }

        None
    }

    /// DMA heap for the frame buffers. The RP1 needs physically contiguous
    /// memory; the RV1126B's VICAP has an IOMMU and the CMA area is small
    /// (32 MiB).
    pub fn frame_buffer_dma_heap(&self) -> &'static str {
        if self.is_rv1126b() {
            return "/dev/dma_heap/system";
        }

        "/dev/dma_heap/linux,cma"
    }

    pub fn mcu_reset_pin(&self) -> u32 {
        26
    }

    pub fn mcu_swd_clk_pin(&self) -> u32 { 
        27
    }

    pub fn mcu_swd_io_pin(&self) -> u32 {
        17
    }

    pub fn mcu_serial_device(&self) -> String {
        // GPIO4: MCU_UART_RX
        // GPIO5: MCU_UART_TX
        if self.config.compute_board_revision() < 6 {
            return match self.config.compute_module() {
                ComputeModuleModel::PI_CM4 | ComputeModuleModel::PI_CM4_LITE => {
                    "/dev/ttyAMA3"
                }
                ComputeModuleModel::PI_CM5 | ComputeModuleModel::PI_CM5_LITE => {
                    "/dev/ttyAMA2"
                }
                _ => panic!()
            }.into();
        }

        // GPIO14: MCU_UART_RX
        // GPIO15: MCU_UART_TX
        "/dev/ttyAMA0".into()
    }

    pub fn accelerometer_i2c_device(&self) -> String {
        "/dev/i2c-1".into()
    }

    pub fn enable_pio_trigger_forwarder(&self) -> bool {
        !self.is_rv1126b() && self.config.compute_board_revision() < 4
    }

    pub fn local_strobe_dimming(&self) -> bool {
        !self.is_rv1126b() && self.config.compute_board_revision() < 6
    }

    pub fn local_rgb_control(&self) -> bool {
        !self.is_rv1126b() && self.config.compute_board_revision() < 8
    }

    pub fn ptp_interface(&self) -> String {
        "eth0".into()
    }
}

