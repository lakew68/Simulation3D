public static class Constants
{
    public static double UnitMass_in_CGS = 2.0e33; // 1 Solar Mass
    public static double UnitLength_in_CGS = 1.5e13; // 1 AU
    public static double UnitTime_in_CGS = 3.14e7 / 36.50; // 10day
    // Acceleration units will be pc/yr^2
    // Force units will be solar mass pc / yr^2
    // solar mass pc / yr^2 = G * solar mass^2 / pc^2
    // G units are pc^3 / yr^2 / Solar mass
    public static double G_in_units = 6.67e-8 / UnitLength_in_CGS / UnitLength_in_CGS / UnitLength_in_CGS * UnitTime_in_CGS * UnitTime_in_CGS * UnitMass_in_CGS;
    public static double softeningScale = 0.001;
    public static float initScale = 0.5f;
}