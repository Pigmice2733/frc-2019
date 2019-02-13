package frc.robot.revapi;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.revapi.types.SPADInfo;

public class VL53L0X {
    private class SequenceStepEnables {
        public boolean tcc, msrc, dss, preRange, finalRange;

        public SequenceStepEnables(boolean tcc, boolean msrc, boolean dss, boolean preRange, boolean finalRange) {
            this.tcc = tcc;
            this.msrc = msrc;
            this.dss = dss;
            this.preRange = preRange;
            this.finalRange = finalRange;
        }
    }

    private class SequenceStepTimeouts {
        public int preRangeVCSELPeriodPclks, finalRangeVCSELPeriodPclks;

        public int msrcDssTccMclks, preRangeMclks, finalRangeMclks;
        public int msrcDssTccUs, preRangeUs, finalRangeUs;

        public SequenceStepTimeouts(int preRangeVCSELPeriodPclks, int finalRangeVCSELPeriodPclks, int msrcDssTccMclks,
                int preRangeMclks, int finalRangeMclks, int msrcDssTccUs, int preRangeUs, int finalRangeUs) {
            this.preRangeVCSELPeriodPclks = preRangeVCSELPeriodPclks;
            this.finalRangeVCSELPeriodPclks = finalRangeVCSELPeriodPclks;
            this.msrcDssTccMclks = msrcDssTccMclks;
            this.preRangeMclks = preRangeMclks;
            this.finalRangeMclks = finalRangeMclks;
            this.msrcDssTccUs = msrcDssTccUs;
            this.preRangeUs = preRangeUs;
            this.finalRangeUs = finalRangeUs;
        }
    }

    private enum vcselPeriodType {
        VCSEL_PERIOD_PRE_RANGE, VCSEL_PERIOD_FINAL_RANGE
    }

    private class Constants {
        private static final int SYSRANGE_START = 0x00;

        private static final int SYSTEM_THRESH_HIGH = 0x0C;
        private static final int SYSTEM_THRESH_LOW = 0x0E;

        private static final int SYSTEM_SEQUENCE_CONFIG = 0x01;
        private static final int SYSTEM_RANGE_CONFIG = 0x09;
        private static final int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;

        private static final int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;

        private static final int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;

        private static final int SYSTEM_INTERRUPT_CLEAR = 0x0B;

        private static final int RESULT_INTERRUPT_STATUS = 0x13;
        private static final int RESULT_RANGE_STATUS = 0x14;

        private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
        private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
        private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
        private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
        private static final int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;

        private static final int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;

        private static final int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;

        private static final int MSRC_CONFIG_CONTROL = 0x60;

        private static final int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
        private static final int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
        private static final int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
        private static final int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;

        private static final int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
        private static final int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
        private static final int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
        private static final int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;

        private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
        private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;

        private static final int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
        private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
        private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;

        private static final int SYSTEM_HISTOGRAM_BIN = 0x81;
        private static final int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
        private static final int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;

        private static final int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
        private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
        private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
        private static final int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;

        private static final int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;

        private static final int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
        private static final int IDENTIFICATION_MODEL_ID = 0xC0;
        private static final int IDENTIFICATION_REVISION_ID = 0xC2;

        private static final int OSC_CALIBRATE_VAL = 0xF8;

        private static final int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
        private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;

        private static final int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
        private static final int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
        private static final int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
        private static final int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;

        private static final int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;

        private static final int ALGO_PHASECAL_LIM = 0x30;
        private static final int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
    }

    private int address = 41;
    private I2C i2c;

    private int measurementTimingBudgetUs = 0;

    // seconds
    private int timeoutStart = 0;
    private int ioTimeout = 0;
    private boolean didTimeout = false;

    private int stopVariable = 0;

    public VL53L0X() {
        i2c = new I2C(I2C.Port.kOnboard, address);
    }

    public void connected() {
        System.out.println(readReg(0xa0));
        System.out.println(readReg(0xc3));
    }

    public void setAddress(int newAddress) {
        writeReg(Constants.I2C_SLAVE_DEVICE_ADDRESS, newAddress & 0x7F);
        address = newAddress;
    }

    public int getAddress() {
        return address;
    }

    public void setTimeout(int timeout) {
        ioTimeout = timeout;
    }

    public int getTimeout() {
        return ioTimeout;
    }

    private int millis() {
        return (int) (Timer.getFPGATimestamp() * 1000.0);
    }

    private boolean writeReg(int reg, int value) {
        return i2c.write(reg & 0xff, value & 0xFF);
    }

    private boolean writeReg16Bit(int reg, int value) {
        byte[] buffer = { (byte) reg, (byte) ((value >> 8) & 0xFF), (byte) (value & 0xFF) };
        return i2c.writeBulk(buffer);
    }

    private boolean writeReg32Bit(int reg, int value) {
        byte[] buffer = { (byte) reg, (byte) ((value >> 24) & 0xFF), (byte) ((value >> 16) & 0xFF),
                (byte) ((value >> 8) & 0xFF), (byte) (value & 0xFF) };
        return i2c.writeBulk(buffer);
    }

    private int readReg(int reg) {
        byte[] buffer = new byte[1];

        i2c.read(address, 1, buffer);

        return (buffer[0] & 0xFF);
    }

    private int readReg16Bit(int reg) {
        byte[] buffer = new byte[2];

        i2c.read(address, 2, buffer);

        int value = (buffer[0] & 0xFF) << 8;
        return value |= (buffer[1] & 0xFF);
    }

    private int readReg32Bit(int reg) {
        byte[] buffer = new byte[4];

        i2c.read(address, 4, buffer);

        int value = (buffer[0] & 0xFF) << 24;
        value |= (buffer[1] & 0xFF) << 16;
        value |= (buffer[2] & 0xFF) << 8;
        return value |= (buffer[3] & 0xFF);
    }

    private void writeMulti(int reg, int[] data, int count) {
        byte[] buffer = new byte[data.length + 1];
        buffer[0] = (byte) reg;

        for (int i = 1; i <= count; i++) {
            buffer[i] = (byte) data[i - 1];
        }

        i2c.writeBulk(buffer, count + 1);
    }

    private int[] readMulti(int reg, int count) {
        byte[] buffer = new byte[count];

        i2c.read(address, count, buffer);

        int[] output = new int[buffer.length];

        for (int i = 0; i < buffer.length; i++) {
            output[i] = buffer[i] & 0xFF;
        }

        return output;
    }

    private void startTimeout() {
        timeoutStart = millis();
    }

    private boolean checkTimeoutExpired() {
        return ioTimeout > 0 && (millis() - timeoutStart) > ioTimeout;
    }

    private int decodeVcselPeriod(int regVal) {
        return (regVal + 1) << 1;
    }

    private int encodeVcselPeriod(int periodPclks) {
        return (periodPclks >> 1) - 1;
    }

    private int calcMacroPeriod(int vcselPeriodPclks) {
        return ((2304 * vcselPeriodPclks * 1655) + 500) / 1000;
    }

    public boolean init(boolean io_2v8) {
        if (io_2v8) {
            writeReg(Constants.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                    readReg(Constants.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
        }

        writeReg(0x88, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        stopVariable = readReg(0x91);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        writeReg(Constants.MSRC_CONFIG_CONTROL, readReg(Constants.MSRC_CONFIG_CONTROL) | 0x12);

        setSignalRateLimit(0.25);

        writeReg(readReg(Constants.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV), 0xFF);

        SPADInfo spadInfo = getSpadInfo();
        if (!spadInfo.ret) {
            return false;
        }

        int[] refSPADMap = readMulti(Constants.GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6);

        writeReg(0xFF, 0x01);
        writeReg(Constants.DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        writeReg(Constants.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        writeReg(0xFF, 0x00);
        writeReg(Constants.GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        int firstSPADToEnable = spadInfo.typeIsAperature ? 12 : 0; // 12 is the first aperture spad
        int SPADsEnabled = 0;

        for (int i = 0; i < 48; i++) {
            if (i < firstSPADToEnable || SPADsEnabled == spadInfo.count) {
                refSPADMap[i / 8] &= ~(1 << (i % 8));
            } else if (((refSPADMap[i / 8] >> (i % 8)) & 0x1) == 1) {
                SPADsEnabled++;
            }
        }

        writeMulti(Constants.GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSPADMap, 6);

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0xFF, 0x00);
        writeReg(0x09, 0x00);
        writeReg(0x10, 0x00);
        writeReg(0x11, 0x00);
        writeReg(0x24, 0x01);
        writeReg(0x25, 0xFF);
        writeReg(0x75, 0x00);
        writeReg(0xFF, 0x01);
        writeReg(0x4E, 0x2C);
        writeReg(0x48, 0x00);
        writeReg(0x30, 0x20);
        writeReg(0xFF, 0x00);
        writeReg(0x30, 0x09);
        writeReg(0x54, 0x00);
        writeReg(0x31, 0x04);
        writeReg(0x32, 0x03);
        writeReg(0x40, 0x83);
        writeReg(0x46, 0x25);
        writeReg(0x60, 0x00);
        writeReg(0x27, 0x00);
        writeReg(0x50, 0x06);
        writeReg(0x51, 0x00);
        writeReg(0x52, 0x96);
        writeReg(0x56, 0x08);
        writeReg(0x57, 0x30);
        writeReg(0x61, 0x00);
        writeReg(0x62, 0x00);
        writeReg(0x64, 0x00);
        writeReg(0x65, 0x00);
        writeReg(0x66, 0xA0);
        writeReg(0xFF, 0x01);
        writeReg(0x22, 0x32);
        writeReg(0x47, 0x14);
        writeReg(0x49, 0xFF);
        writeReg(0x4A, 0x00);
        writeReg(0xFF, 0x00);
        writeReg(0x7A, 0x0A);
        writeReg(0x7B, 0x00);
        writeReg(0x78, 0x21);
        writeReg(0xFF, 0x01);
        writeReg(0x23, 0x34);
        writeReg(0x42, 0x00);
        writeReg(0x44, 0xFF);
        writeReg(0x45, 0x26);
        writeReg(0x46, 0x05);
        writeReg(0x40, 0x40);
        writeReg(0x0E, 0x06);
        writeReg(0x20, 0x1A);
        writeReg(0x43, 0x40);
        writeReg(0xFF, 0x00);
        writeReg(0x34, 0x03);
        writeReg(0x35, 0x44);
        writeReg(0xFF, 0x01);
        writeReg(0x31, 0x04);
        writeReg(0x4B, 0x09);
        writeReg(0x4C, 0x05);
        writeReg(0x4D, 0x04);
        writeReg(0xFF, 0x00);
        writeReg(0x44, 0x00);
        writeReg(0x45, 0x20);
        writeReg(0x47, 0x08);
        writeReg(0x48, 0x28);
        writeReg(0x67, 0x00);
        writeReg(0x70, 0x04);
        writeReg(0x71, 0x01);
        writeReg(0x72, 0xFE);
        writeReg(0x76, 0x00);
        writeReg(0x77, 0x00);
        writeReg(0xFF, 0x01);
        writeReg(0x0D, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0x01, 0xF8);
        writeReg(0xFF, 0x01);
        writeReg(0x8E, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        writeReg(Constants.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        writeReg(Constants.GPIO_HV_MUX_ACTIVE_HIGH, readReg(Constants.GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // low
        writeReg(Constants.SYSTEM_INTERRUPT_CLEAR, 0x01);
        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, 0xE8);
        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, 0x01);

        if (!performSingleRefCalibration(0x40)) {
            return false;
        }

        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, 0x02);

        if (!performSingleRefCalibration(0x00)) {
            return false;
        }

        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, 0xE8);

        return true;
    }

    public boolean setSignalRateLimit(double limit_Mcps) {
        if (limit_Mcps < 0 || limit_Mcps > 511.99) {
            return false;
        }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeReg16Bit(Constants.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int) (limit_Mcps * (1 << 7)));
        return true;
    }

    public double getSignalRateLimit() {
        return (double) readReg16Bit(Constants.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
    }

    public boolean setMeasurementTimingBudget(int budgetUs) {
        SequenceStepEnables enables = new SequenceStepEnables(false, false, false, false, false);
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts(0, 0, 0, 0, 0, 0, 0, 0);

        final int StartOverhead = 1320;
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        final int MinTimingBudget = 20000;

        if (budgetUs < MinTimingBudget) {
            return false;
        }

        int usedBudgetUs = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        if (enables.tcc) {
            usedBudgetUs += (timeouts.msrcDssTccUs + TccOverhead);
        }

        if (enables.dss) {
            usedBudgetUs += 2 * (timeouts.msrcDssTccUs + DssOverhead);
        } else if (enables.msrc) {
            usedBudgetUs += (timeouts.msrcDssTccUs + MsrcOverhead);
        }

        if (enables.preRange) {
            usedBudgetUs += (timeouts.preRangeUs + PreRangeOverhead);
        }

        if (enables.finalRange) {
            usedBudgetUs += FinalRangeOverhead;

            if (usedBudgetUs > budgetUs) {
                // "Requested timeout too big."
                return false;
            }

            int finalRangeTimeoutUs = budgetUs - usedBudgetUs;

            int finalRangeTimeoutMclks = timeoutMicrosecondsToMclks(finalRangeTimeoutUs,
                    timeouts.finalRangeVCSELPeriodPclks);

            if (enables.preRange) {
                finalRangeTimeoutMclks += timeouts.preRangeMclks;
            }

            writeReg16Bit(Constants.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(finalRangeTimeoutMclks));

            measurementTimingBudgetUs = budgetUs;
        }
        return true;
    }

    public int getMeasurementTimingBudget() {
        SequenceStepEnables enables = new SequenceStepEnables(false, false, false, false, false);
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts(0, 0, 0, 0, 0, 0, 0, 0);

        final int StartOverhead = 1910;
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        int budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        if (enables.tcc) {
            budget_us += (timeouts.msrcDssTccUs + TccOverhead);
        }

        if (enables.dss) {
            budget_us += 2 * (timeouts.msrcDssTccUs + DssOverhead);
        } else if (enables.msrc) {
            budget_us += (timeouts.msrcDssTccUs + MsrcOverhead);
        }

        if (enables.preRange) {
            budget_us += (timeouts.preRangeUs + PreRangeOverhead);
        }

        if (enables.finalRange) {
            budget_us += (timeouts.finalRangeUs + FinalRangeOverhead);
        }

        measurementTimingBudgetUs = budget_us; // store for internal reuse
        return budget_us;
    }

    public boolean setVcselPulsePeriod(vcselPeriodType type, int period_pclks) {
        int vcselPeriodReg = encodeVcselPeriod(period_pclks);

        SequenceStepEnables enables = new SequenceStepEnables(false, false, false, false, false);
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts(0, 0, 0, 0, 0, 0, 0, 0);

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        if (type == vcselPeriodType.VCSEL_PERIOD_PRE_RANGE) {
            // "Set phase check limits"
            switch (period_pclks) {
            case 12:
                writeReg(Constants.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                break;

            case 14:
                writeReg(Constants.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                break;

            case 16:
                writeReg(Constants.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                break;

            case 18:
                writeReg(Constants.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                break;

            default:
                return false;
            }
            writeReg(Constants.PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            writeReg(Constants.PRE_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg);

            int new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.preRangeUs, period_pclks);

            writeReg16Bit(Constants.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

            int new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrcDssTccUs, period_pclks);

            writeReg(Constants.MSRC_CONFIG_TIMEOUT_MACROP,
                    (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

            // set_sequence_step_timeout() end
        } else if (type == vcselPeriodType.VCSEL_PERIOD_FINAL_RANGE) {
            switch (period_pclks) {
            case 8:
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(Constants.GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                writeReg(Constants.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                writeReg(0xFF, 0x01);
                writeReg(Constants.ALGO_PHASECAL_LIM, 0x30);
                writeReg(0xFF, 0x00);
                break;

            case 10:
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(Constants.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(Constants.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                writeReg(0xFF, 0x01);
                writeReg(Constants.ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 12:
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(Constants.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(Constants.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                writeReg(0xFF, 0x01);
                writeReg(Constants.ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 14:
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                writeReg(Constants.FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(Constants.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(Constants.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                writeReg(0xFF, 0x01);
                writeReg(Constants.ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            default:
                return false;
            }

            writeReg(Constants.FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg);

            int new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.finalRangeUs, period_pclks);

            if (enables.preRange) {
                new_final_range_timeout_mclks += timeouts.preRangeMclks;
            }

            writeReg16Bit(Constants.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));

        } else {
            return false;
        }

        setMeasurementTimingBudget(measurementTimingBudgetUs);

        int sequence_config = readReg(Constants.SYSTEM_SEQUENCE_CONFIG);
        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, 0x02);
        performSingleRefCalibration(0x0);
        writeReg(Constants.SYSTEM_SEQUENCE_CONFIG, sequence_config);

        return true;
    }

    public int getVcselPulsePeriod(vcselPeriodType type) {
        if (type == vcselPeriodType.VCSEL_PERIOD_PRE_RANGE) {
            return decodeVcselPeriod(readReg(Constants.PRE_RANGE_CONFIG_VCSEL_PERIOD));
        } else if (type == vcselPeriodType.VCSEL_PERIOD_FINAL_RANGE) {
            return decodeVcselPeriod(readReg(Constants.FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        } else {
            return 255;
        }
    }

    public void startContinuous(int periodMs) {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stopVariable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        if (periodMs != 0) {
            int oscCalibrateVal = readReg16Bit(Constants.OSC_CALIBRATE_VAL);

            if (oscCalibrateVal != 0) {
                periodMs *= oscCalibrateVal;
            }

            writeReg32Bit(Constants.SYSTEM_INTERMEASUREMENT_PERIOD, periodMs);

            writeReg(Constants.SYSRANGE_START, 0x04);
        } else {
            writeReg(Constants.SYSRANGE_START, 0x02);
        }
    }

    public void stopContinuous() {
        writeReg(Constants.SYSRANGE_START, 0x01);

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, 0x00);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
    }

    public int readRangeContinuousMillimeters() {
        startTimeout();
        while ((readReg(Constants.RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if (checkTimeoutExpired()) {
                didTimeout = true;
                return 65535;
            }
        }

        int range = readReg16Bit(Constants.RESULT_RANGE_STATUS + 10);

        writeReg(Constants.SYSTEM_INTERRUPT_CLEAR, 0x01);

        return range;
    }

    public int readRangeSingleMillimeters() {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stopVariable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        writeReg(Constants.SYSRANGE_START, 0x01);

        // "Wait until start bit has been cleared"
        startTimeout();
        while ((readReg(Constants.SYSRANGE_START) & 0x01) == 1) {
            if (checkTimeoutExpired()) {
                didTimeout = true;
                return 65535;
            }
        }

        return readRangeContinuousMillimeters();
    }

    public boolean timeoutOccurred() {
        boolean tmp = didTimeout;
        didTimeout = false;
        return tmp;
    }

    public SPADInfo getSpadInfo() {
        int tmp;

        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) | 0x04);
        writeReg(0xFF, 0x07);
        writeReg(0x81, 0x01);
        writeReg(0x80, 0x01);
        writeReg(0x94, 0x6b);
        writeReg(0x83, 0x00);

        startTimeout();

        while (readReg(0x83) == 0x00) {
            if (checkTimeoutExpired()) {
                didTimeout = true;
                return new SPADInfo(false, 0, false);
            }
        }

        writeReg(0x83, 0x01);
        tmp = readReg(0x92);

        writeReg(0x81, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) & ~0x04);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        return new SPADInfo(true, tmp & 0x7F, ((tmp >> 7) & 0x01) == 1);
    }

    public void getSequenceStepEnables(SequenceStepEnables enables) {
        int sequence_config = readReg(Constants.SYSTEM_SEQUENCE_CONFIG);

        enables.tcc = ((sequence_config >> 4) & 0x1) == 1;
        enables.dss = ((sequence_config >> 3) & 0x1) == 1;
        enables.msrc = ((sequence_config >> 2) & 0x1) == 1;
        enables.preRange = ((sequence_config >> 6) & 0x1) == 1;
        enables.finalRange = ((sequence_config >> 7) & 0x1) == 1;
    }

    public void getSequenceStepTimeouts(SequenceStepEnables enables, SequenceStepTimeouts timeouts) {
        timeouts.preRangeVCSELPeriodPclks = getVcselPulsePeriod(vcselPeriodType.VCSEL_PERIOD_PRE_RANGE);

        timeouts.msrcDssTccMclks = readReg(Constants.MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts.msrcDssTccUs = timeoutMclksToMicroseconds(timeouts.msrcDssTccMclks, timeouts.preRangeVCSELPeriodPclks);

        timeouts.preRangeMclks = decodeTimeout(readReg16Bit(Constants.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts.preRangeUs = timeoutMclksToMicroseconds(timeouts.preRangeMclks, timeouts.preRangeVCSELPeriodPclks);

        timeouts.finalRangeVCSELPeriodPclks = getVcselPulsePeriod(vcselPeriodType.VCSEL_PERIOD_FINAL_RANGE);

        timeouts.finalRangeMclks = decodeTimeout(readReg16Bit(Constants.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (enables.preRange) {
            timeouts.finalRangeMclks -= timeouts.preRangeMclks;
        }

        timeouts.finalRangeUs = timeoutMclksToMicroseconds(timeouts.finalRangeMclks,
                timeouts.finalRangeVCSELPeriodPclks);
    }

    public int decodeTimeout(int regVal) {
        // LSByte * 2^MSByte + 1"
        return (int) ((regVal & 0x00FF) << (int) ((regVal & 0xFF00) >> 8)) + 1;
    }

    public int encodeTimeout(int timeoutMclks) {
        // LSByte * 2^MSByte + 1"

        int lsByte = 0;
        int msByte = 0;

        if (timeoutMclks > 0) {
            lsByte = timeoutMclks - 1;

            while ((lsByte & 0xFFFFFF00) > 0) {
                lsByte >>= 1;
                msByte++;
            }

            return (msByte << 8) | (lsByte & 0xFF);
        } else {
            return 0;
        }
    }

    public int timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks) {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    public int timeoutMicrosecondsToMclks(int timeout_period_us, int vcsel_period_pclks) {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }

    public boolean performSingleRefCalibration(int vhvInitByte) {
        writeReg(Constants.SYSRANGE_START, 0x01 | vhvInitByte);

        startTimeout();
        while ((readReg(Constants.RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if (checkTimeoutExpired()) {
                return false;
            }
        }

        writeReg(Constants.SYSTEM_INTERRUPT_CLEAR, 0x01);

        writeReg(Constants.SYSRANGE_START, 0x00);

        return true;
    }
}
