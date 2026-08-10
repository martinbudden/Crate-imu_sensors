# Notes

To run tests on the PC:

```sh
cargo test --no-default-features --features "std,libm" --target x86_64-unknown-linux-gnu
```

or

```sh
cargo test-host
```

To check with a given target use:

```sh
cargo check --target thumbv8m.main-none-eabihf
```

```rust
#[inline]
fn map_acc_gyro(&self, buf: [u8; 12]) -> (Vector3f32, Vector3f32) {
    let gyro_bytes = [buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]];
    let acc_bytes = [buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]];

    let acc = Vector3f32::from_le_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
    let gyro = Vector3f32::from_le_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;

    ImuAxisOrder::map_acc_gyro(axis_order, acc, gyro)
}
#[inline]
pub fn map_acc_gyro(&self, buf: [u8; 12]) -> (Vector3f32, Vector3f32) {
    let (acc_slice, gyro_slice) = buf.split_at(6);

    let acc = Vector3f32::from_le_slice_6(acc_slice)* self.common.acc_scale - self.common.acc_offset;
    let gyro = Vector3f32::from_le_slice_6(gyro_slice) * self.common.gyro_scale - self.common.gyro_offset;

    ImuAxisOrder::map_acc_gyro(axis_order, acc, gyro)
}

#[inline]
pub fn map_acc_gyro2(&self, buf: [u8; 12]) -> (Vector3f32, Vector3f32) {
    let (acc_slice, gyro_slice) = buf.split_at(6);

    let acc = Vector3f32::from_le_slice_6(acc_slice);
    let gyro = Vector3f32::from_le_slice_6(gyro_slice);

    ImuAxisOrder::map_acc_gyro(axis_order, acc, gyro)
}
        /*let acc_buf = [buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]];
        let _temperature = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro_buf = [buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]];
        let acc = Vector3f32::from_be_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_be_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;*/

        /*let acc_slice = &buf[0..6];
        // let temperature_slice = &slice[6..8];
        let gyro_slice = &buf[8..14];
        let acc = Vector3f32::from_be_slice_6(acc_slice) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_be_slice_6(gyro_slice) * self.common.gyro_scale - self.common.gyro_offset;*/


    /*async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.bus_write_read(I2C_ADDRESS, write, read).await
    }
    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        // Custom hardware handling before the transaction
        self.toggle_chip_select_low();

        // Perform a custom transaction sequence
        let result = self.bus().bus_write_read(0x42, write, read).await;

        // Custom hardware handling after the transaction
        self.toggle_chip_select_high();

        result.map_err(Self::Error::from)
    }*/
```
