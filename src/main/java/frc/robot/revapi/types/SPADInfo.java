package frc.robot.revapi.types;

public class SPADInfo {
    public boolean ret;
    public int count;
    public boolean typeIsAperature;

    public SPADInfo(boolean ret, int count, boolean typeIsAperature) {
        this.ret = ret;
        this.count = count;
        this.typeIsAperature = typeIsAperature;
    }
}