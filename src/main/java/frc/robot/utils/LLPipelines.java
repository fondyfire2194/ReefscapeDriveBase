// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class LLPipelines {

    public enum pipelines {
        APRILTAGALL0(0, pipelinetype.fiducialmarkers), // all tags
        CORALDET1(1, pipelinetype.detector),
        ALGAEDET1(1, pipelinetype.detector);
      

        public static final pipelines values[] = values();

        private pipelinetype type;

        public String pipelineTypeName;

        private int number;

        private pipelines(int number, pipelinetype type) {
            this.number = number;
            this.type = type;
        }

    }

    public enum pipelinetype {
        color_retroreflective,
        grip,
        python,
        fiducialmarkers,
        classifier,
        detector;

        public static final pipelinetype values[] = values();
    }

}
