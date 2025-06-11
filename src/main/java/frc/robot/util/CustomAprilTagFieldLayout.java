package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


import java.util.ArrayList;
import java.util.List;

import java.util.HashSet;

public class CustomAprilTagFieldLayout {

    public static AprilTagFieldLayout getFieldLayout(int[] TagIds) {

        AprilTagFieldLayout fullFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        List<AprilTag> allAprilTags = fullFieldLayout.getTags();

        HashSet<Integer> desiredTagIdsSet = new HashSet<>();
        for (int id : TagIds) {
            desiredTagIdsSet.add(id);
        }

        List<AprilTag> filteredAprilTags = new ArrayList<>();

        for (AprilTag tag : allAprilTags) {
            if (desiredTagIdsSet.contains(tag.ID)) {
                filteredAprilTags.add(tag);
            }
        }

        return new AprilTagFieldLayout(filteredAprilTags,fullFieldLayout.getFieldLength(),fullFieldLayout.getFieldWidth());
    }
}