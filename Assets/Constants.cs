public static class Constants
{
    public static double UnitMass_in_CGS = 2.0e33; // 1 Solar Mass
    public static double UnitLength_in_CGS = 3.1e17; // .1 pc
    public static double UnitTime_in_CGS = 3.14e9; // 100 years
    // Acceleration units will be pc/yr^2
    // Force units will be solar mass pc / yr^2
    // solar mass pc / yr^2 = G * solar mass^2 / pc^2
    // G units are pc^3 / yr^2 / Solar mass
    public static double G_in_units = 6.67e-8 / UnitLength_in_CGS / UnitLength_in_CGS / UnitLength_in_CGS * UnitTime_in_CGS * UnitTime_in_CGS * UnitMass_in_CGS;
    public static double softeningScale = 0.001;
}