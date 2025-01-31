package frc.robot.subsystems.utils;

public enum SparkModels {
    SparkMax(0),
    SparkFlex(1),
    Unknown(255);

    public final int id;

    SparkModels(int id) {
      this.id = id;
    }

    public static SparkModels fromId(int id) {
      for (SparkModels model : values()) {
        if (model.id == id) return model;
      }
      return Unknown;
    }
}
