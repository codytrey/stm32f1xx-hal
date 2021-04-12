use super::*;



/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2cSlave<I2C, PINS> {
    nb: I2c<I2C, PINS>,
    start_timeout: u32,
    start_retries: u8,
    addr_timeout: u32,
    data_timeout: u32,
    state: State,
}

impl<PINS> BlockingI2cSlave<I2C1, PINS> {
    /// Creates a blocking I2C1 object on pins PB6 and PB7 or PB8 and PB9 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c1_slave(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        BlockingI2cSlave::<I2C1, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<I2C, PINS> BlockingI2cSlave<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    fn _i2c(
        i2c: I2C,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut I2C::Bus,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        blocking_i2c_slave(
            I2c::<I2C, _>::_i2c_slave(i2c, pins, mode, clocks, apb),
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

/// Generates a blocking I2C instance from a universal I2C object
fn blocking_i2c_slave<I2C, PINS>(
    i2c: I2c<I2C, PINS>,
    clocks: Clocks,
    start_timeout_us: u32,
    start_retries: u8,
    addr_timeout_us: u32,
    data_timeout_us: u32,
) -> BlockingI2cSlave<I2C, PINS> {
    let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
    BlockingI2cSlave {
        nb: i2c,
        start_timeout: start_timeout_us * sysclk_mhz,
        start_retries,
        addr_timeout: addr_timeout_us * sysclk_mhz,
        data_timeout: data_timeout_us * sysclk_mhz,
        state: State::None,
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock> + Enable + Reset,
    I2C::Bus: GetBusFreq,
{
    /// Configures the I2C peripheral to work in master mode
    fn _i2c_slave(i2c: I2C, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut I2C::Bus) -> Self {
        I2C::enable(apb);
        I2C::reset(apb);

        let pclk1 = I2C::Bus::get_frequency(&clocks).0;

        assert!(mode.get_frequency().0 <= 400_000);

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            pclk1,
        };
        i2c.init_slave();
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init_slave(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = (self.pclk1 / 1000000) as u16;

        self.i2c
            .cr2
            .write(|w| unsafe { w.freq().bits(pclk1_mhz as u8) });
        self.i2c.cr1.write(|w| w.pe().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz + 1) as u8));
                self.i2c.ccr.write(|w| unsafe {
                    w.ccr().bits(((self.pclk1 / (freq.0 * 2)) as u16).max(4))
                });
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz * 300 / 1000 + 1) as u8));

                self.i2c.ccr.write(|w| {
                    let (freq, duty) = match duty_cycle {
                        &DutyCycle::Ratio2to1 => {
                            (((self.pclk1 / (freq.0 * 3)) as u16).max(1), false)
                        }
                        &DutyCycle::Ratio16to9 => {
                            (((self.pclk1 / (freq.0 * 25)) as u16).max(1), true)
                        }
                    };

                    unsafe { w.ccr().bits(freq).duty().bit(duty).f_s().set_bit() }
                });
            }
        };

        self.own_7_bit_address_setup(0x20);

        self.i2c.cr1.modify(|_, w| w.pe().set_bit());

        // Slave needs to acknowledge on receiving bytes
        // set it after enabling Peripheral i.e. PE = 1
        self.i2c.cr1.modify(|_,w| {
            w.ack().set_bit();
            //w.nostretch().set_bit();
            w.engc().set_bit();
            w
        });
    }

    fn own_7_bit_address_setup(&mut self, address: u8) {
        self.i2c.oar1.write(|w| {
            w.addmode().clear_bit();
            w.add().bits((address as u16) << 1);
            w
        });
    }

    fn get_last_event(&mut self) -> Option<Event> {
        let sr1 = self.i2c.sr1.read();
        let sr2 = self.i2c.sr2.read();
        
        Some(if sr2.busy().bit_is_set()
            && sr1.addr().bit_is_set()
        {
            Event::ReceiverAddressMatched
        }
        else if sr2.tra().bit_is_set()
            && sr2.busy().bit_is_set()
            && sr1.tx_e().bit_is_set()
            && sr1.addr().bit_is_set()
        {
            Event::TrasmitterAddressMatched
        }
        else if sr2.busy().bit_is_set()
            && sr1.rx_ne().bit_is_set()
        {
            Event::ByteReceived
        }
        else if sr2.tra().bit_is_set()
            && sr2.busy().bit_is_set()
            && sr1.tx_e().bit_is_set()
            && sr1.btf().bit_is_set()
        {
            Event::ByteTransmitted
        }
        else if sr2.tra().bit_is_set()
            && sr2.busy().bit_is_set()
            && sr1.tx_e().bit_is_set()
        {
            Event::ByteTransmitting
        }
        else if sr1.stopf().bit_is_set() {
            Event::StopDetected
        }
        else {
            return None;
        })
    }

    fn clear_flags(&mut self) {
        // Full clear sequence:
        // if (ADDR == 1) {READ SR1; READ SR2}
        if self.i2c.sr1.read().addr().bit_is_set() {
            let sr1 = self.i2c.sr1.read();
            let sr2 = self.i2c.sr2.read();
            log::info!("sr1: {:016b}", sr1.bits());
            log::info!("sr2: {:016b}", sr2.bits());
        }
        // Full clear sequence:
        // if (STOPF == 1) {READ SR1; WRITE CR1}
        while self.i2c.sr1.read().stopf().bit_is_set() {
            let _sr1 = self.i2c.sr1.read();
            self.i2c.cr1.modify(|_,w| {
                w.ack().set_bit();
                //w.nostretch().set_bit();
                //w.engc().set_bit();
                w
            });
        }
    }

    fn send_data(&mut self, data: u8) {
        self.i2c.dr.write(|w| w.dr().bits(data))
    }

    fn receive_data(&mut self) -> u8 {
        self.i2c.dr.read().dr().bits()
    }

    fn it_status_clear(&mut self) {
        let sr1 = self.i2c.sr1.read();

        if sr1.smbalert().bit_is_set()
            || sr1.timeout().bit_is_set()
            || sr1.pecerr().bit_is_set()
            || sr1.ovr().bit_is_set()
            || sr1.af().bit_is_set()
            || sr1.arlo().bit_is_set()
            || sr1.berr().bit_is_set()
        {
            self.i2c.sr1.modify(|_,w| {
                w.smbalert().clear_bit()
                   .timeout().clear_bit()
                   .pecerr().clear_bit()
                   .ovr().clear_bit()
                   .af().clear_bit()
                   .arlo().clear_bit()
                   .berr().clear_bit()
            });
        }
    }
}

enum State {
    None,
    AddrWrite,
    AddrByte,
}

#[derive(Debug)]
enum Event {
    ReceiverAddressMatched,
    ByteReceived,
    TrasmitterAddressMatched,
    ByteTransmitting,
    ByteTransmitted,
    StopDetected,
}

impl<I2C, PINS> BlockingI2cSlave<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    pub fn listen(&mut self) {
        // Handle interrupt errors
        self.nb.it_status_clear();
        // Reading last event
        let ev = self.nb.get_last_event();
        
        if let Some(ev) = ev {
            match ev {
                // Master has sent the slave address to send data to the slave
                Event::ReceiverAddressMatched => {
                    //log::info!("ReceiverAddressMatched");
                }
                // Master has sent a byte to the slave
                Event::ByteReceived => {
                    let b = self.nb.receive_data();
                    //log::info!("ByteReceived: {:X?}", b);
                    log::info!("{:X?}", b);
                }
                // Master has sent the slave address to read data from the slave
                Event::TrasmitterAddressMatched => {
                    //log::info!("TrasmitterAddressMatched");
                    self.nb.send_data(0x0A);
                }
                // Master wants to read another byte of data from the slave
                Event::ByteTransmitted | Event::ByteTransmitting => {
                    //log::info!("ByteTransmitted");
                    self.nb.send_data(0x0B);
                }
                // Master has STOP sent
                Event::StopDetected => {
                    //log::info!("StopDetected");
                    self.nb.clear_flags();
                }
                e => todo!("{:?}", e),
            }
        }
    }
}
