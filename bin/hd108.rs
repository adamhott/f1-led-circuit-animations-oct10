use embedded_hal_async::spi::SpiBus;
use heapless07::Vec;

pub struct HD108<SPI> {
    pub spi: SPI,
}

impl<SPI> HD108<SPI>
where
    SPI: SpiBus<u8>,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    // Function to create an LED frame
    fn create_led_frame(red: u16, green: u16, blue: u16) -> [u8; 8] {
        let start_code: u8 = 0b1;
        let red_gain: u8 = 0b00010; // Regulation level 2 - 2.24 mA
        let green_gain: u8 = 0b00010; // Regulation level 2 - 2.24 mA
        let blue_gain: u8 = 0b00010; // Regulation level 2 - 2.24 mA

        // Combine the gain values into a 15-bit number
        let current_gain =
            ((red_gain as u16) << 10) | ((green_gain as u16) << 5) | (blue_gain as u16);

        // The first byte contains the start code and the 7 most significant bits of the current gain
        let first_byte = (start_code << 7) | ((current_gain >> 8) as u8 & 0x7F);

        // The second byte contains the remaining 8 bits of the current gain
        let second_byte = (current_gain & 0xFF) as u8;

        [
            first_byte,           // Start code and part of current gain
            second_byte,          // Remaining current gain bits
            (red >> 8) as u8,     // High byte of red
            (red & 0xFF) as u8,   // Low byte of red
            (green >> 8) as u8,   // High byte of green
            (green & 0xFF) as u8, // Low byte of green
            (blue >> 8) as u8,    // High byte of blue
            (blue & 0xFF) as u8,  // Low byte of blue
        ]
    }

    pub async fn set_off(&mut self) -> Result<(), SPI::Error> {
        // At least 128 bits of zeros for the start frame
        let start_frame = [0x00; 16];

        // Create data frames for all 96 LEDs
        let mut data: Vec<u8, 796> = Vec::new();
        data.extend_from_slice(&start_frame).unwrap();

        // Set all LEDs to off
        let off_led_frame = Self::create_led_frame(0x0000, 0x0000, 0x0000);
        for _ in 0..96 {
            data.extend_from_slice(&off_led_frame).unwrap();
        }

        // Additional clock pulses equal to the number of LEDs in the strip
        let additional_clocks = [0x00; 12];
        data.extend_from_slice(&additional_clocks).unwrap();

        // Write the data to the SPI bus
        self.spi.write(&data).await?;

        Ok(())
    }

    pub async fn set_leds(&mut self, leds: &[(usize, u8, u8, u8)]) -> Result<(), SPI::Error> {
        // At least 128 bits of zeros for the start frame
        let start_frame = [0x00; 16];

        // Create data frames for all 96 LEDs
        let mut data: Vec<u8, 796> = Vec::new();
        data.extend_from_slice(&start_frame).unwrap();

        // Set the specified LEDs to the given colors and all others to off
        for i in 1..=96 {
            if let Some(&(_led_num, red, green, blue)) =
                leds.iter().find(|&&(led_num, _, _, _)| led_num == i)
            {
                // Convert the 8-bit RGB values to 16-bit values
                let red = ((red as u16) << 8) | (red as u16);
                let green = ((green as u16) << 8) | (green as u16);
                let blue = ((blue as u16) << 8) | (blue as u16);

                let led_frame = Self::create_led_frame(red, green, blue);
                data.extend_from_slice(&led_frame).unwrap();
            } else {
                let off_led_frame = Self::create_led_frame(0x0000, 0x0000, 0x0000); // LED off
                data.extend_from_slice(&off_led_frame).unwrap();
            }
        }

        // Additional clock pulses equal to the number of LEDs in the strip
        let additional_clocks = [0x00; 12];
        data.extend_from_slice(&additional_clocks).unwrap();

        // Write the data to the SPI bus
        self.spi.write(&data).await?;

        Ok(())
    }
}
