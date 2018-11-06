package org.firstinspires.ftc.teamcode.gamefield;

import org.firstinspires.ftc.teamcode.skills.GoldPosition;

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

public class MineralLineUp {
    private ArrayList<MineralObject> lineUp = new ArrayList();
    private GoldPosition goldPosition = GoldPosition.None;
    private boolean goldFound = false;
    private int goldIndex = -1;

    public ArrayList getLineUp() {
        return lineUp;
    }

    public void addSample(MineralObject obj){
        this.lineUp.add(obj);
        if (!isGoldFound() && obj.getLabel().equals(LABEL_GOLD_MINERAL)){
            setGoldFound(true);
        }
    }

    public int compute(){
        Collections.sort(lineUp);
        if (isGoldFound()){
            for(int i = 0; i < lineUp.size(); i++){
                MineralObject obj = lineUp.get(i);
                if (obj.getLabel().equals(LABEL_GOLD_MINERAL)){
                    setGoldIndex(i);
                    break;
                }
            }
        }
        return getGoldIndex();
    }

    public void setLineUp(ArrayList lineUp) {
        this.lineUp = lineUp;
    }

    public GoldPosition getGoldPosition() {
        return goldPosition;
    }

    public void setGoldPosition(GoldPosition goldPosition) {
        this.goldPosition = goldPosition;
    }

    public boolean isGoldFound() {
        return goldFound;
    }

    public void setGoldFound(boolean goldFound) {
        this.goldFound = goldFound;
    }

    public int getGoldIndex() {
        return goldIndex;
    }

    public void setGoldIndex(int goldIndex) {
        this.goldIndex = goldIndex;
    }
}
