import array
import math
import struct

import machine
import typing


def init_float_array(size) -> array.array:
    return array.array('f', (0 for _ in range(size)))


def init_int_array(size) -> array.array:
    return array.array('i', (0 for _ in range(size)))


class RefreshRate:
    """Enum-like class for MLX90640's refresh rate."""
    REFRESH_0_5_HZ = 0b000  # 0.5Hz
    REFRESH_1_HZ = 0b001  # 1Hz
    REFRESH_2_HZ = 0b010  # 2Hz
    REFRESH_4_HZ = 0b011  # 4Hz
    REFRESH_8_HZ = 0b100  # 8Hz
    REFRESH_16_HZ = 0b101  # 16Hz
    REFRESH_32_HZ = 0b110  # 32Hz
    REFRESH_64_HZ = 0b111  # 64Hz


class I2CDevice:
    """
    Represents a single I2C device and manages locking the bus and the device
    address.
    """

    def __init__(self, i2c, device_address, probe=True):
        self.i2c = i2c
        self.device_address = device_address

        if probe:
            self._probe_for_device()

    def read_into(self, buf, *, start=0, end=None):
        """Read into buffer from the device without allocation."""
        if end is None:
            end = len(buf)

        self.i2c.readfrom_into(self.device_address, buf, start=start, end=end)

    def write(self, buf):
        """Write buffer to the device."""
        self.i2c.writeto(self.device_address, buf)

    def write_then_read_into(self, out_buffer, in_buffer, *, out_start=0, out_end=None, in_start=0, in_end=None):
        """Write to the device and read from the device into a buffer."""
        if out_end is None:
            out_end = len(out_buffer)

        if in_end is None:
            in_end = len(in_buffer)

        self.i2c.writeto(self.device_address, memoryview(out_buffer)[out_start:out_end], False)
        self.i2c.readfrom_into(self.device_address, memoryview(in_buffer)[in_start:in_end])

    def _probe_for_device(self):
        """Probe for the device, ensuring it is responding on the bus."""

        try:
            self.i2c.writeto(self.device_address, b'')
        except OSError:
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.device_address, result)
            except OSError:
                raise ValueError(f'No I2C device at address: 0x{self.device_address:x}')


class MLX90640:
    """Interface to the MLX90640 temperature sensor."""

    ee_data = init_int_array(834)
    i2c_read_len = 128
    scale_alpha = 0.000001
    mlx90640_deviceid1 = 0x2407
    openair_ta_shift = 8

    def __init__(self, i2c_bus: machine.I2C, address: int = 0x33) -> None:
        self.inbuf = bytearray(2 * self.i2c_read_len)
        self.addrbuf = bytearray(2)
        self.i2c_device = I2CDevice(i2c_bus, address)
        self.mlx90640_frame = init_int_array(834)
        self._i2c_read_words(0x2400, self.ee_data)

        # Attributes initialized through extraction methods
        self.k_vdd = 0
        self.vdd25 = 0
        self.kv_ptat = 0
        self.kt_ptat = 0
        self.v_ptat25 = 0
        self.alpha_ptat = 0
        self.gain_ee = 0
        self.tgc = 0
        self.resolution_ee = 0
        self.ks_ta = 0
        self.ct = [0] * 4
        self.ks_to = [0] * 5
        self.cp_alpha = [0, 0]
        self.cp_offset = [0, 0]
        self.alpha = None
        self.alpha_scale = 0
        self.offset = None
        self.kta = None
        self.kta_scale = 0
        self.kv = None
        self.kv_scale = 0
        self.il_chess_c = [0, 0, 0]
        self.broken_pixels = set()
        self.outlier_pixels = set()
        self.calibration_mode_ee = 0

        self._extract_parameters()

    @property
    def serial_number(self) -> typing.List[int]:
        """3-item tuple of hex values that are unique to each MLX90640"""
        serial_words = [0, 0, 0]
        self._i2c_read_words(self.mlx90640_deviceid1, serial_words)
        return serial_words

    @property
    def refresh_rate(self) -> int:
        """How fast the MLX90640 will spit out data. Start at lowest speed in
        RefreshRate and then slowly increase I2C clock rate and rate until you
        max out. The sensor does not like it if the I2C host cannot 'keep up'!"""
        control_register = [0]
        self._i2c_read_words(0x800D, control_register)
        return (control_register[0] >> 7) & 0x07

    @refresh_rate.setter
    def refresh_rate(self, rate: int) -> None:
        control_register = [0]
        value = (rate & 0x7) << 7
        self._i2c_read_words(0x800D, control_register)
        value |= control_register[0] & 0xFC7F
        self._i2c_write_word(0x800D, value)

    def get_frame(self, framebuf: typing.List[int]) -> None:
        """Request both 'halves' of a frame from the sensor, merge them
        and calculate the temperature in C for each of 32x24 pixels. Placed
        into the 768-element array passed in!"""
        emissivity = 0.95

        status = self._get_frame_data()

        if status < 0:
            raise RuntimeError('Frame data error')

        tr = self._get_ta() - self.openair_ta_shift

        self._calculate_to(emissivity, tr, framebuf)

    def _get_frame_data(self) -> int:
        data_ready = 0
        cnt = 0
        status_register = [0]
        control_register = [0]

        while data_ready == 0:
            self._i2c_read_words(0x8000, status_register)
            data_ready = status_register[0] & 0x0008

        while (data_ready != 0) and (cnt < 5):
            self._i2c_write_word(0x8000, 0x0030)
            self._i2c_read_words(0x0400, self.mlx90640_frame, end=832)

            self._i2c_read_words(0x8000, status_register)
            data_ready = status_register[0] & 0x0008
            cnt += 1

        if cnt > 4:
            raise RuntimeError('Too many retries')

        self._i2c_read_words(0x800D, control_register)
        self.mlx90640_frame[832] = control_register[0]
        self.mlx90640_frame[833] = status_register[0] & 0x0001
        return self.mlx90640_frame[833]

    def _get_ta(self) -> float:
        vdd = self._get_vdd()

        ptat = self.mlx90640_frame[800]
        if ptat > 32767:
            ptat -= 65536

        ptat_art = self.mlx90640_frame[768]
        if ptat_art > 32767:
            ptat_art -= 65536
        ptat_art = (ptat / (ptat * self.alpha_ptat + ptat_art)) * math.pow(2, 18)

        ta = ptat_art / (1 + self.kv_ptat * (vdd - 3.3)) - self.v_ptat25
        ta = ta / self.kt_ptat + 25
        return ta

    def _get_vdd(self) -> int:
        vdd = self.mlx90640_frame[810]

        if vdd > 32767:
            vdd -= 65536

        resolution_ram = (self.mlx90640_frame[832] & 0x0C00) >> 10
        resolution_correction = math.pow(2, self.resolution_ee) / math.pow(2, resolution_ram)
        vdd = (resolution_correction * vdd - self.vdd25) / self.k_vdd + 3.3

        return vdd

    def _calculate_to(self, emissivity: float, tr: float, result: typing.List[float]) -> None:
        sub_page = self.mlx90640_frame[833]
        alpha_corr_r = [0] * 4
        ir_data_cp = [0, 0]

        vdd = self._get_vdd()
        ta = self._get_ta()

        ta4 = (ta + 273.15) ** 4
        tr4 = (tr + 273.15) ** 4
        ta_tr = tr4 - (tr4 - ta4) / emissivity

        kta_scale = math.pow(2, self.kta_scale)
        kv_scale = math.pow(2, self.kv_scale)
        alpha_scale = math.pow(2, self.alpha_scale)

        alpha_corr_r[0] = 1 / (1 + self.ks_to[0] * 40)
        alpha_corr_r[1] = 1
        alpha_corr_r[2] = 1 + self.ks_to[1] * self.ct[2]
        alpha_corr_r[3] = alpha_corr_r[2] * (1 + self.ks_to[2] * (self.ct[3] - self.ct[2]))

        gain = self.mlx90640_frame[778]
        if gain > 32767:
            gain -= 65536
        gain = self.gain_ee / gain

        mode = (self.mlx90640_frame[832] & 0x1000) >> 5

        ir_data_cp[0] = self.mlx90640_frame[776]
        ir_data_cp[1] = self.mlx90640_frame[808]
        for i in range(2):
            if ir_data_cp[i] > 32767:
                ir_data_cp[i] -= 65536
            ir_data_cp[i] *= gain

        ir_data_cp[0] -= self.cp_offset[0] * (1 + self.cp_kta * (ta - 25)) * (1 + self.cp_kv * (vdd - 3.3))
        if mode == self.calibration_mode_ee:
            ir_data_cp[1] -= self.cp_offset[1] * (1 + self.cp_kta * (ta - 25)) * (1 + self.cp_kv * (vdd - 3.3))
        else:
            ir_data_cp[1] -= (self.cp_offset[1] + self.il_chess_c[0]) * (1 + self.cp_kta * (ta - 25)) * (
                1 + self.cp_kv * (vdd - 3.3))

        for pixel_number in range(768):
            if self._is_pixel_bad(pixel_number):
                result[pixel_number] = -273.15
                continue

            il_pattern = pixel_number // 32 - (pixel_number // 64) * 2
            conversion_pattern = ((pixel_number + 2) // 4 - (pixel_number + 3) // 4 + (
                pixel_number + 1) // 4 - pixel_number // 4) * (1 - 2 * il_pattern)

            if mode == 0:
                pattern = il_pattern
            else:
                chess_pattern = il_pattern ^ (pixel_number - (pixel_number // 2) * 2)
                pattern = chess_pattern

            if pattern == sub_page:
                ir_data = self.mlx90640_frame[pixel_number]

                if ir_data > 32767:
                    ir_data -= 65536

                ir_data *= gain

                kta = self.kta[pixel_number] / kta_scale
                kv = self.kv[pixel_number] / kv_scale
                ir_data -= self.offset[pixel_number] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3))

                if mode != self.calibration_mode_ee:
                    ir_data += self.il_chess_c[2] * (2 * il_pattern - 1) - self.il_chess_c[1] * conversion_pattern

                ir_data = ir_data - self.tgc * ir_data_cp[sub_page]
                ir_data /= emissivity

                alpha_compensated = (
                    (self.scale_alpha * alpha_scale / self.alpha[pixel_number])
                    * (1 + self.ks_ta * (ta - 25))
                )

                sx = math.sqrt(math.sqrt(
                    alpha_compensated
                    * alpha_compensated
                    * alpha_compensated
                    * (ir_data + alpha_compensated * ta_tr)
                ))
                to = math.sqrt(math.sqrt(
                    (ir_data / (alpha_compensated * (1 - self.ks_to[1] * 273.15) + sx) + ta_tr)
                )) - 273.15

                if to < self.ct[1]:
                    torange = 0
                elif to < self.ct[2]:
                    torange = 1
                elif to < self.ct[3]:
                    torange = 2
                else:
                    torange = 3

                to = math.sqrt(math.sqrt(
                    ir_data / (
                        alpha_compensated
                        * alpha_corr_r[torange]
                        * (1 + self.ks_to[torange] * (to - self.ct[torange]))
                    ) + ta_tr
                )) - 273.15

                result[pixel_number] = to

    def _extract_parameters(self) -> None:
        self._extract_vdd_parameters()
        self._extract_ptat_parameters()
        self._extract_gain_parameters()
        self._extract_tgc_parameters()
        self._extract_resolution_parameters()
        self._extract_ks_ta_parameters()
        self._extract_ks_to_parameters()
        self._extract_cp_parameters()
        self._extract_alpha_parameters()
        self._extract_offset_parameters()
        self._extract_kta_pixel_parameters()
        self._extract_kv_pixel_parameters()
        self._extract_cilc_parameters()
        self._extract_deviating_pixels()

    def _extract_vdd_parameters(self) -> None:
        # extract VDD
        self.k_vdd = (self.ee_data[51] & 0xFF00) >> 8
        if self.k_vdd > 127:
            self.k_vdd -= 256  # convert to signed
        self.k_vdd *= 32
        self.vdd25 = self.ee_data[51] & 0x00FF
        self.vdd25 = ((self.vdd25 - 256) << 5) - 8192

    def _extract_ptat_parameters(self) -> None:
        self.kv_ptat = (self.ee_data[50] & 0xFC00) >> 10
        if self.kv_ptat > 31:
            self.kv_ptat -= 64
        self.kv_ptat /= 4096
        self.kt_ptat = self.ee_data[50] & 0x03FF
        if self.kt_ptat > 511:
            self.kt_ptat -= 1024
        self.kt_ptat /= 8
        self.v_ptat25 = self.ee_data[49]
        self.alpha_ptat = (self.ee_data[16] & 0xF000) / math.pow(2, 14) + 8

    def _extract_gain_parameters(self) -> None:
        self.gain_ee = self.ee_data[48]
        if self.gain_ee > 32767:
            self.gain_ee -= 65536

    def _extract_tgc_parameters(self) -> None:
        self.tgc = self.ee_data[60] & 0x00FF
        if self.tgc > 127:
            self.tgc -= 256
        self.tgc /= 32

    def _extract_resolution_parameters(self) -> None:
        self.resolution_ee = (self.ee_data[56] & 0x3000) >> 12

    def _extract_ks_ta_parameters(self) -> None:
        self.ks_ta = (self.ee_data[60] & 0xFF00) >> 8
        if self.ks_ta > 127:
            self.ks_ta -= 256
        self.ks_ta /= 8192

    def _extract_ks_to_parameters(self) -> None:
        step = ((self.ee_data[63] & 0x3000) >> 12) * 10
        self.ct[0] = -40
        self.ct[1] = 0
        self.ct[2] = (self.ee_data[63] & 0x00F0) >> 4
        self.ct[3] = (self.ee_data[63] & 0x0F00) >> 8
        self.ct[2] *= step
        self.ct[3] = self.ct[2] + self.ct[3] * step

        ks_to_scale = (self.ee_data[63] & 0x000F) + 8
        ks_to_scale = 1 << ks_to_scale

        self.ks_to[0] = self.ee_data[61] & 0x00FF
        self.ks_to[1] = (self.ee_data[61] & 0xFF00) >> 8
        self.ks_to[2] = self.ee_data[62] & 0x00FF
        self.ks_to[3] = (self.ee_data[62] & 0xFF00) >> 8

        for i in range(4):
            if self.ks_to[i] > 127:
                self.ks_to[i] -= 256
            self.ks_to[i] /= ks_to_scale
        self.ks_to[4] = -0.
        # extract CP
        offset_sp = [0] * 2
        alpha_sp = [0] * 2

        alpha_scale = ((self.ee_data[32] & 0xF000) >> 12) + 27

        offset_sp[0] = self.ee_data[58] & 0x03FF
        if offset_sp[0] > 511:
            offset_sp[0] -= 1024

        offset_sp[1] = (self.ee_data[58] & 0xFC00) >> 10
        if offset_sp[1] > 31:
            offset_sp[1] -= 64
        offset_sp[1] += offset_sp[0]

        alpha_sp[0] = self.ee_data[57] & 0x03FF
        if alpha_sp[0] > 511:
            alpha_sp[0] -= 1024
        alpha_sp[0] /= math.pow(2, alpha_scale)

        alpha_sp[1] = (self.ee_data[57] & 0xFC00) >> 10
        if alpha_sp[1] > 31:
            alpha_sp[1] -= 64
        alpha_sp[1] = (1 + alpha_sp[1] / 128) * alpha_sp[0]

        cp_kta = self.ee_data[59] & 0x00FF
        if cp_kta > 127:
            cp_kta -= 256
        kta_scale1 = ((self.ee_data[56] & 0x00F0) >> 4) + 8
        self.cp_kta = cp_kta / math.pow(2, kta_scale1)

        cp_kv = (self.ee_data[59] & 0xFF00) >> 8
        if cp_kv > 127:
            cp_kv -= 256
        kv_scale = (self.ee_data[56] & 0x0F00) >> 8
        self.cp_kv = cp_kv / math.pow(2, kv_scale)

        self.cp_alpha[0] = alpha_sp[0]
        self.cp_alpha[1] = alpha_sp[1]
        self.cp_offset[0] = offset_sp[0]
        self.cp_offset[1] = offset_sp[1]

    def _extract_cp_parameters(self):
        # Compensation Pixel (CP) parameters extraction from EEPROM data
        offset_sp = [0] * 2
        alpha_sp = [0] * 2

        alpha_scale = ((self.ee_data[32] & 0xF000) >> 12) + 27

        offset_sp[0] = self.ee_data[58] & 0x03FF
        if offset_sp[0] > 511:
            offset_sp[0] -= 1024

        offset_sp[1] = (self.ee_data[58] & 0xFC00) >> 10
        if offset_sp[1] > 31:
            offset_sp[1] -= 64
        offset_sp[1] += offset_sp[0]

        alpha_sp[0] = self.ee_data[57] & 0x03FF
        if alpha_sp[0] > 511:
            alpha_sp[0] -= 1024
        alpha_sp[0] /= math.pow(2, alpha_scale)

        alpha_sp[1] = (self.ee_data[57] & 0xFC00) >> 10
        if alpha_sp[1] > 31:
            alpha_sp[1] -= 64
        alpha_sp[1] = (1 + alpha_sp[1] / 128) * alpha_sp[0]

        cp_kta = self.ee_data[59] & 0x00FF
        if cp_kta > 127:
            cp_kta -= 256
        kta_scale1 = ((self.ee_data[56] & 0x00F0) >> 4) + 8
        self.cp_kta = cp_kta / math.pow(2, kta_scale1)

        cp_kv = (self.ee_data[59] & 0xFF00) >> 8
        if cp_kv > 127:
            cp_kv -= 256
        kv_scale = (self.ee_data[56] & 0x0F00) >> 8
        self.cp_kv = cp_kv / math.pow(2, kv_scale)

        self.cp_alpha = alpha_sp
        self.cp_offset = offset_sp

    def _extract_alpha_parameters(self) -> None:
        # extract alpha
        acc_rem_scale = self.ee_data[32] & 0x000F
        acc_column_scale = (self.ee_data[32] & 0x00F0) >> 4
        acc_row_scale = (self.ee_data[32] & 0x0F00) >> 8
        alpha_scale = ((self.ee_data[32] & 0xF000) >> 12) + 30
        alpha_ref = self.ee_data[33]
        acc_row = init_int_array(24)
        acc_column = init_int_array(32)
        alpha_temp = init_float_array(768)

        for i in range(6):
            p = i * 4
            acc_row[p + 0] = self.ee_data[34 + i] & 0x000F
            acc_row[p + 1] = (self.ee_data[34 + i] & 0x00F0) >> 4
            acc_row[p + 2] = (self.ee_data[34 + i] & 0x0F00) >> 8
            acc_row[p + 3] = (self.ee_data[34 + i] & 0xF000) >> 12

        for i in range(24):
            if acc_row[i] > 7:
                acc_row[i] -= 16

        for i in range(8):
            p = i * 4
            acc_column[p + 0] = self.ee_data[40 + i] & 0x000F
            acc_column[p + 1] = (self.ee_data[40 + i] & 0x00F0) >> 4
            acc_column[p + 2] = (self.ee_data[40 + i] & 0x0F00) >> 8
            acc_column[p + 3] = (self.ee_data[40 + i] & 0xF000) >> 12

        for i in range(32):
            if acc_column[i] > 7:
                acc_column[i] -= 16

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                alpha_temp[p] = (self.ee_data[64 + p] & 0x03F0) >> 4
                if alpha_temp[p] > 31:
                    alpha_temp[p] -= 64
                alpha_temp[p] *= 1 << acc_rem_scale
                alpha_temp[p] += (
                    alpha_ref
                    + (acc_row[i] << acc_row_scale)
                    + (acc_column[j] << acc_column_scale)
                )
                alpha_temp[p] /= math.pow(2, alpha_scale)
                alpha_temp[p] -= self.tgc * (self.cp_alpha[0] + self.cp_alpha[1]) / 2
                alpha_temp[p] = self.scale_alpha / alpha_temp[p]

        temp = max(alpha_temp)

        alpha_scale = 0
        while temp < 32768:
            temp *= 2
            alpha_scale += 1

        for i in range(768):
            temp = alpha_temp[i] * math.pow(2, alpha_scale)
            alpha_temp[i] = int(temp + 0.5)

        self.alpha = alpha_temp
        self.alpha_scale = alpha_scale

    def _extract_offset_parameters(self) -> None:
        # extract offset
        occ_row = [0] * 24
        occ_column = [0] * 32

        occ_rem_scale = self.ee_data[16] & 0x000F
        occ_column_scale = (self.ee_data[16] & 0x00F0) >> 4
        occ_row_scale = (self.ee_data[16] & 0x0F00) >> 8
        offset_ref = self.ee_data[17]
        if offset_ref > 32767:
            offset_ref -= 65536

        for i in range(6):
            p = i * 4
            occ_row[p + 0] = self.ee_data[18 + i] & 0x000F
            occ_row[p + 1] = (self.ee_data[18 + i] & 0x00F0) >> 4
            occ_row[p + 2] = (self.ee_data[18 + i] & 0x0F00) >> 8
            occ_row[p + 3] = (self.ee_data[18 + i] & 0xF000) >> 12

        for i in range(24):
            if occ_row[i] > 7:
                occ_row[i] -= 16

        for i in range(8):
            p = i * 4
            occ_column[p + 0] = self.ee_data[24 + i] & 0x000F
            occ_column[p + 1] = (self.ee_data[24 + i] & 0x00F0) >> 4
            occ_column[p + 2] = (self.ee_data[24 + i] & 0x0F00) >> 8
            occ_column[p + 3] = (self.ee_data[24 + i] & 0xF000) >> 12

        for i in range(32):
            if occ_column[i] > 7:
                occ_column[i] -= 16

        self.offset = init_float_array(768)

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                self.offset[p] = (self.ee_data[64 + p] & 0xFC00) >> 10
                if self.offset[p] > 31:
                    self.offset[p] -= 64
                self.offset[p] *= 1 << occ_rem_scale
                self.offset[p] += (
                    offset_ref
                    + (occ_row[i] << occ_row_scale)
                    + (occ_column[j] << occ_column_scale)
                )

    def _extract_kta_pixel_parameters(self):
        # Extract KtaPixel
        kta_rc = [0] * 4
        kta_temp = init_float_array(768)

        kta_ro_co = (self.ee_data[54] & 0xFF00) >> 8
        if kta_ro_co > 127:
            kta_ro_co -= 256
        kta_rc[0] = kta_ro_co

        kta_re_co = self.ee_data[54] & 0x00FF
        if kta_re_co > 127:
            kta_re_co -= 256
        kta_rc[2] = kta_re_co

        kta_ro_ce = (self.ee_data[55] & 0xFF00) >> 8
        if kta_ro_ce > 127:
            kta_ro_ce -= 256
        kta_rc[1] = kta_ro_ce

        kta_re_ce = self.ee_data[55] & 0x00FF
        if kta_re_ce > 127:
            kta_re_ce -= 256
        kta_rc[3] = kta_re_ce

        kta_scale1 = ((self.ee_data[56] & 0x00F0) >> 4) + 8
        kta_scale2 = self.ee_data[56] & 0x000F

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = 2 * (p // 32 - (p // 64) * 2) + p % 2
                kta_temp[p] = (self.ee_data[64 + p] & 0x000E) >> 1
                if kta_temp[p] > 3:
                    kta_temp[p] -= 8
                kta_temp[p] *= 1 << kta_scale2
                kta_temp[p] += kta_rc[split]
                kta_temp[p] /= math.pow(2, kta_scale1)

        temp = max(abs(k) for k in kta_temp)

        kta_scale1 = 0
        while temp < 64:
            temp *= 2
            kta_scale1 += 1

        for i in range(768):
            temp = kta_temp[i] * math.pow(2, kta_scale1)
            if temp < 0:
                kta_temp[i] = int(temp - 0.5)
            else:
                kta_temp[i] = int(temp + 0.5)

        self.kta = kta_temp
        self.kta_scale = kta_scale1

    def _extract_kv_pixel_parameters(self):
        # Extract KvPixel
        kv_t = [0] * 4
        kv_temp = init_float_array(768)

        kv_ro_co = (self.ee_data[52] & 0xF000) >> 12
        if kv_ro_co > 7:
            kv_ro_co -= 16
        kv_t[0] = kv_ro_co

        kv_re_co = (self.ee_data[52] & 0x0F00) >> 8
        if kv_re_co > 7:
            kv_re_co -= 16
        kv_t[2] = kv_re_co

        kv_ro_ce = (self.ee_data[52] & 0x00F0) >> 4
        if kv_ro_ce > 7:
            kv_ro_ce -= 16
        kv_t[1] = kv_ro_ce

        kv_re_ce = self.ee_data[52] & 0x000F
        if kv_re_ce > 7:
            kv_re_ce -= 16
        kv_t[3] = kv_re_ce

        kv_scale = (self.ee_data[56] & 0x0F00) >> 8

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = 2 * (p // 32 - (p // 64) * 2) + p % 2
                kv_temp[p] = kv_t[split]
                kv_temp[p] /= math.pow(2, kv_scale)

        temp = max(abs(kv) for kv in kv_temp)

        kv_scale = 0
        while temp < 64:
            temp *= 2
            kv_scale += 1

        for i in range(768):
            temp = kv_temp[i] * math.pow(2, kv_scale)
            if temp < 0:
                kv_temp[i] = int(temp - 0.5)
            else:
                kv_temp[i] = int(temp + 0.5)

        self.kv = kv_temp
        self.kv_scale = kv_scale

    def _extract_cilc_parameters(self):
        # Extract CILC parameters
        self.calibration_mode_ee = (self.ee_data[10] & 0x0800) >> 4
        self.calibration_mode_ee = self.calibration_mode_ee ^ 0x80

        il_chess_c = [0] * 3
        il_chess_c[0] = self.ee_data[53] & 0x003F
        if il_chess_c[0] > 31:
            il_chess_c[0] -= 64
        il_chess_c[0] /= 16.0

        il_chess_c[1] = (self.ee_data[53] & 0x07C0) >> 6
        if il_chess_c[1] > 15:
            il_chess_c[1] -= 32
        il_chess_c[1] /= 2.0

        il_chess_c[2] = (self.ee_data[53] & 0xF800) >> 11
        if il_chess_c[2] > 15:
            il_chess_c[2] -= 32
        il_chess_c[2] /= 8.0

        self.il_chess_c = il_chess_c

    def _extract_deviating_pixels(self):
        # Extract information about deviating pixels
        pix_cnt = 0
        while (
            (pix_cnt < 768)
            and (len(self.broken_pixels) < 5)
            and (len(self.outlier_pixels) < 5)
        ):
            if self.ee_data[pix_cnt + 64] == 0:
                self.broken_pixels.add(pix_cnt)
            elif (self.ee_data[pix_cnt + 64] & 0x0001) != 0:
                self.outlier_pixels.add(pix_cnt)
            pix_cnt += 1

        if len(self.broken_pixels) > 4:
            raise RuntimeError('More than 4 broken pixels')
        if len(self.outlier_pixels) > 4:
            raise RuntimeError('More than 4 outlier pixels')
        if (len(self.broken_pixels) + len(self.outlier_pixels)) > 4:
            raise RuntimeError('More than 4 faulty pixels')

        for broken_pixel1, broken_pixel2 in self._unique_list_pairs(self.broken_pixels):
            if self._are_pixels_adjacent(broken_pixel1, broken_pixel2):
                raise RuntimeError('Adjacent broken pixels')

        for outlier_pixel1, outlier_pixel2 in self._unique_list_pairs(self.outlier_pixels):
            if self._are_pixels_adjacent(outlier_pixel1, outlier_pixel2):
                raise RuntimeError('Adjacent outlier pixels')

        for broken_pixel in self.broken_pixels:
            for outlier_pixel in self.outlier_pixels:
                if self._are_pixels_adjacent(broken_pixel, outlier_pixel):
                    raise RuntimeError('Adjacent broken and outlier pixels')

    def _unique_list_pairs(self, input_list: typing.List[int]) -> typing.Tuple[int, int]:
        for i, list_value1 in enumerate(input_list):
            for list_value2 in input_list[i + 1:]:
                yield list_value1, list_value2

    def _are_pixels_adjacent(self, pix1: int, pix2: int) -> bool:
        pix_pos_dif = pix1 - pix2

        if -34 < pix_pos_dif < -30:
            return True
        if -2 < pix_pos_dif < 2:
            return True
        if 30 < pix_pos_dif < 34:
            return True

        return False

    def _is_pixel_bad(self, pixel: int) -> bool:
        return pixel in self.broken_pixels or pixel in self.outlier_pixels

    def _i2c_write_word(self, write_address: int, data: int) -> None:
        cmd = bytearray(4)
        cmd[0] = write_address >> 8
        cmd[1] = write_address & 0x00FF
        cmd[2] = data >> 8
        cmd[3] = data & 0x00FF
        data_check = [0]

        self.i2c_device.write(cmd)
        self._i2c_read_words(write_address, data_check)

    def _i2c_read_words(
        self,
        addr: int,
        buffer: typing.Union[int, typing.List[int]],
        *,
        end: typing.Optional[int] = None,
    ) -> None:
        if end is None:
            remaining_words = len(buffer)
        else:
            remaining_words = end
        offset = 0

        while remaining_words:
            self.addrbuf[0] = addr >> 8  # MSB
            self.addrbuf[1] = addr & 0xFF  # LSB
            read_words = min(remaining_words, self.i2c_read_len)
            self.i2c_device.write_then_read_into(
                self.addrbuf,
                self.inbuf,
                in_end=read_words * 2,
            )

            outwords = struct.unpack(
                '>' + 'H' * read_words,
                self.inbuf if len(self.inbuf) == read_words * 2 else self.inbuf[:read_words * 2],
            )
            for i, w in enumerate(outwords):
                buffer[offset + i] = w

            offset += read_words
            remaining_words -= read_words
            addr += read_words
