package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

import java.util.List;

public class HubBulkRead {
    public static LynxModule CONTROL_HUB, EXPANSION_HUB;
    private List<LynxModule> allHubs;

    private static HubBulkRead instance = null;
    private boolean justControlHub = false;

    private LynxModule.BulkCachingMode currentCachingMode = LynxModule.BulkCachingMode.AUTO;

    public static HubBulkRead getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new HubBulkRead(hardwareMap);
        }

        return instance;
    }

    public static HubBulkRead getInstance(HardwareMap hardwareMap, LynxModule.BulkCachingMode cachingMode) {
        if (instance == null) {
            instance = new HubBulkRead(hardwareMap, cachingMode);
        }

        return instance;
    }

    private void construct(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);

        setMode(currentCachingMode);

        CONTROL_HUB = allHubs.get(0);
        if (allHubs.size() > 1) { EXPANSION_HUB = allHubs.get(1); }
        else { justControlHub = true; }
    }

    public HubBulkRead(HardwareMap hardwareMap){ construct(hardwareMap); }

    public HubBulkRead(HardwareMap hardwareMap, LynxModule.BulkCachingMode cachingMode){
        this.currentCachingMode = cachingMode;
        construct(hardwareMap);
    }

    public void setMode(LynxModule.BulkCachingMode cachingMode) {
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(cachingMode);
        }
    }

    public void clearCache(Enums.Hubs type) {
        if (currentCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            switch (type) {
                case ALL: {
                    for (LynxModule hub : allHubs) {
                        hub.clearBulkCache();
                    }
                }
                break;
                case CONTROL_HUB: {
                    CONTROL_HUB.clearBulkCache();
                }
                break;
                case EXPANSION_HUB: {
                    if (!justControlHub) {
                        EXPANSION_HUB.clearBulkCache();
                    }
                }
                break;
                default: {
                }
            }
        }
    }

    public LynxModule.BulkCachingMode getCurrentCachingMode() { return currentCachingMode; }
}
