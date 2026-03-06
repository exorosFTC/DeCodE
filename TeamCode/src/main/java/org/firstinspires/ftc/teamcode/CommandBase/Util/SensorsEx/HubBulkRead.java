package org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx;

import static android.graphics.Color.BLACK;
import static android.graphics.Color.BLUE;
import static android.graphics.Color.MAGENTA;
import static android.graphics.Color.RED;
import static android.graphics.Color.WHITE;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.aBitOfTrolling;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class HubBulkRead {
    public enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

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

    public void clearCache(Hubs type) {
        if (currentCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            switch (type) {
                case ALL: {
                    for (LynxModule hub : allHubs) {
                        if (aBitOfTrolling) if (autoOnBlue) hub.setConstant(BLUE); else hub.setConstant(RED);
                        hub.clearBulkCache();
                    }

                }
                break;
                case CONTROL_HUB: {
                    if (aBitOfTrolling) if (autoOnBlue) CONTROL_HUB.setConstant(BLUE); else CONTROL_HUB.setConstant(RED);
                    CONTROL_HUB.clearBulkCache();
                }
                break;
                case EXPANSION_HUB: {
                    if (!justControlHub) {
                        if (aBitOfTrolling) if (autoOnBlue) EXPANSION_HUB.setConstant(BLUE); else EXPANSION_HUB.setConstant(RED);
                        EXPANSION_HUB.clearBulkCache();
                    }
                }
                break;
                default: {}
            }
        }
    }

    public LynxModule.BulkCachingMode getCurrentCachingMode() { return currentCachingMode; }
}
