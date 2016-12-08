#include "sf_gps.h"

/* 360 is 90 degrees in (1 degree / 4 LSB), which is where we use it */
#define ANGLE_CUTOFF 360

/* 270 degrees in (1 degree / 4 LSB) */
#define ANGLE_OFFSET 1080

/*Latitude and Longitude for about where we are so we can translate degrees to meters in Lat and Long values */
#define SLO_LATITUDE 35
#define SLO_LONGITUDE -120 


/* Takes the raw IMU compass bearing, x-axis linear acceleration, and
 * y-axis linear acceleration values and converts them into valid accelerations
 * in the longitudinal and latitudinal directions, placing them into their
 * respective fields of the gps_t struct pointed to by &gps
 *
 * Note that the passed compass bearing is (1 degree / 16 LSBs) and passed
 * accelerations are (1 m/s / 100 LSB) */
void convert_acc(int compass_bearing, float acc_x, float acc_y, gps_t *gps) {
	/* Convert bearing to (1 degree / 4 LSB), as this is the resolution of the LUTs */
	compass_bearing /= 4;

	int y_angle, x_angle;

	/* Angle for calculations for x is just compass bearing */
	x_angle = compass_bearing;

	/* Y is 90 degrees out of phase from X and must be piecewise constructed */
	if (compass_bearing >= ANGLE_CUTOFF)
		y_angle = compass_bearing - ANGLE_CUTOFF;
	else
		y_angle = compass_bearing + ANGLE_OFFSET;

	float lat_x_component, lat_y_component, long_x_component, long_y_component;

	/* Extract x components of latitude and longitude */
	lat_x_component = cos_LUT[x_angle] * acc_x;
	long_x_component = sin_LUT[x_angle] * acc_x;

	/* Extract y components of latitude and longitude */
	lat_y_component = cos_LUT[y_angle] * acc_y;
	long_y_component = sin_LUT[y_angle] * acc_y;

	/* Combine components and assign to respective fields in passed struct */
	gps->acc_lat = lat_x_component + lat_y_component;
	gps->acc_long = long_x_component + long_y_component;
}

/*takes the string version of the latitude longitude from the gps, which is in degrees minutes seconds form
  and turns it into some actual floating points, which are then put into the fields
  gps->latitude
  gps->longitude
  Example of a string that might come in: 4807.038,N,01131.000,E
  48 degree 7.038 minutes North
  11 degree 31.000 minutes East
*/
void convert_lat_long(char *stringVers, gps_t *gps) {
    float latDegree;    
    float longDegree;    
    float latMinute;    
    float longMinute;    
    float latitude;
    float longitude;

    char NS;
    char EW;
    
    //char[10] tempBuf; 
    char ptrLoc[10];
/*
    ptrLoc = stringVers;

    snprintf(tempBuf, LAT_DEGREE_LEN, "%s\0", stringVers);
    longDegree = atof(tempBuf);
    stringVers += LONG_DEGREE_LEN;

    snprintf(tempBuf, MINUTE_LEN, "%s\0", stringVers);
    longMinute = atof(tempBuf);
    stringVers += MINUTE_LEN + 2;

    NS = *(ptrLoc++);
*/
    //Go forward till you reach the ., then back track 2 to get where the minutes start
    /* This doesn't seem necessary, latitude degree will be 2 units, longitude 3
    while (ptrLoc != '.') {
        ptrLoc++;
    }
    ptrLoc -= 2;
    snprintf(tempBuf, ptrLoc - stringVers, "%s", stringVers);
    latDegree = atof(tempBuf);

    stringVers = ptrLoc;
    while (ptrLoc != ',') {
        ptrLoc++;
    }

    snprintf(tempBuf, ptrLoc - stringVers, "%s", stringVers);
    latMinute = atof(tempBuf);

    ptrLoc++;
    stringVers = ptrLoc;
    */
/*
    //Now they're both at the first character of the longitude
    snprintf(tempBuf, LONG_DEGREE_LEN, "%s\0", stringVers);
    longDegree = atof(tempBuf);
    stringVers += LONG_DEGREE_LEN;

    snprintf(tempBuf, MINUTE_LEN, "%s\0", stringVers);
    longMinute = atof(tempBuf);
    stringVers += MINUTE_LEN + 2;

    EW = *stringVers;
    
    latitude = latDegree + (latMinute / 60);  
    longitude = longDegree + (longMinute / 60);  

    if (NS == 'S') {
        latitude = -latitude; 
    }
    if (EW == 'W') {
        longitude = -longitude; 
    }
 */
    gps->latitude = latitude;
    gps->longitude = longitude;
}


const float sin_LUT[LUT_SIZE] =
{
		 0.000000, 0.004363, 0.008727, 0.013090, 0.017452,
		 0.021815, 0.026177, 0.030539, 0.034899, 0.039260,
		 0.043619, 0.047978, 0.052336, 0.056693, 0.061049,
		 0.065403, 0.069756, 0.074108, 0.078459, 0.082808,
		 0.087156, 0.091502, 0.095846, 0.100188, 0.104528,
		 0.108867, 0.113203, 0.117537, 0.121869, 0.126199,
		 0.130526, 0.134851, 0.139173, 0.143493, 0.147809,
		 0.152123, 0.156434, 0.160743, 0.165048, 0.169349,
		 0.173648, 0.177943, 0.182235, 0.186524, 0.190809,
		 0.195090, 0.199368, 0.203642, 0.207912, 0.212178,
		 0.216440, 0.220697, 0.224951, 0.229200, 0.233445,
		 0.237686, 0.241922, 0.246153, 0.250380, 0.254602,
		 0.258819, 0.263031, 0.267238, 0.271440, 0.275637,
		 0.279829, 0.284015, 0.288196, 0.292372, 0.296541,
		 0.300706, 0.304864, 0.309017, 0.313164, 0.317305,
		 0.321439, 0.325568, 0.329691, 0.333807, 0.337917,
		 0.342020, 0.346117, 0.350207, 0.354291, 0.358368,
		 0.362438, 0.366501, 0.370557, 0.374606, 0.378648,
		 0.382683, 0.386711, 0.390731, 0.394744, 0.398749,
		 0.402747, 0.406737, 0.410719, 0.414693, 0.418660,
		 0.422618, 0.426569, 0.430511, 0.434445, 0.438371,
		 0.442289, 0.446198, 0.450098, 0.453990, 0.457874,
		 0.461748, 0.465614, 0.469471, 0.473320, 0.477159,
		 0.480989, 0.484809, 0.488621, 0.492423, 0.496216,
		 0.500000, 0.503774, 0.507538, 0.511293, 0.515038,
		 0.518773, 0.522498, 0.526214, 0.529919, 0.533614,
		 0.537299, 0.540974, 0.544639, 0.548293, 0.551937,
		 0.555570, 0.559193, 0.562805, 0.566406, 0.569997,
		 0.573576, 0.577145, 0.580703, 0.584249, 0.587785,
		 0.591309, 0.594823, 0.598324, 0.601815, 0.605294,
		 0.608761, 0.612217, 0.615661, 0.619094, 0.622514,
		 0.625923, 0.629320, 0.632705, 0.636078, 0.639439,
		 0.642787, 0.646124, 0.649448, 0.652760, 0.656059,
		 0.659346, 0.662620, 0.665881, 0.669130, 0.672367,
		 0.675590, 0.678801, 0.681998, 0.685183, 0.688354,
		 0.691513, 0.694658, 0.697790, 0.700909, 0.704015,
		 0.707107, 0.710185, 0.713250, 0.716302, 0.719340,
		 0.722364, 0.725374, 0.728371, 0.731354, 0.734322,
		 0.737277, 0.740218, 0.743145, 0.746057, 0.748956,
		 0.751840, 0.754709, 0.757565, 0.760406, 0.763232,
		 0.766044, 0.768842, 0.771624, 0.774392, 0.777146,
		 0.779884, 0.782608, 0.785317, 0.788011, 0.790689,
		 0.793353, 0.796002, 0.798635, 0.801254, 0.803857,
		 0.806444, 0.809017, 0.811574, 0.814115, 0.816641,
		 0.819152, 0.821647, 0.824126, 0.826590, 0.829037,
		 0.831469, 0.833886, 0.836286, 0.838670, 0.841039,
		 0.843391, 0.845728, 0.848048, 0.850352, 0.852640,
		 0.854912, 0.857167, 0.859406, 0.861629, 0.863835,
		 0.866025, 0.868199, 0.870356, 0.872496, 0.874620,
		 0.876727, 0.878817, 0.880891, 0.882947, 0.884987,
		 0.887011, 0.889017, 0.891006, 0.892979, 0.894934,
		 0.896873, 0.898794, 0.900698, 0.902585, 0.904455,
		 0.906308, 0.908143, 0.909961, 0.911762, 0.913545,
		 0.915311, 0.917060, 0.918791, 0.920505, 0.922201,
		 0.923879, 0.925540, 0.927184, 0.928809, 0.930417,
		 0.932008, 0.933580, 0.935135, 0.936672, 0.938191,
		 0.939692, 0.941176, 0.942641, 0.944089, 0.945518,
		 0.946930, 0.948324, 0.949699, 0.951056, 0.952396,
		 0.953717, 0.955020, 0.956305, 0.957571, 0.958820,
		 0.960050, 0.961262, 0.962455, 0.963630, 0.964787,
		 0.965926, 0.967046, 0.968148, 0.969231, 0.970296,
		 0.971342, 0.972370, 0.973379, 0.974370, 0.975342,
		 0.976296, 0.977231, 0.978147, 0.979045, 0.979925,
		 0.980785, 0.981627, 0.982450, 0.983255, 0.984041,
		 0.984808, 0.985556, 0.986286, 0.986996, 0.987688,
		 0.988361, 0.989016, 0.989651, 0.990268, 0.990866,
		 0.991445, 0.992005, 0.992546, 0.993068, 0.993572,
		 0.994056, 0.994522, 0.994968, 0.995396, 0.995805,
		 0.996195, 0.996565, 0.996917, 0.997250, 0.997564,
		 0.997859, 0.998135, 0.998392, 0.998630, 0.998848,
		 0.999048, 0.999229, 0.999391, 0.999534, 0.999657,
		 0.999762, 0.999848, 0.999914, 0.999962, 0.999990,
		 1.000000, 0.999990, 0.999962, 0.999914, 0.999848,
		 0.999762, 0.999657, 0.999534, 0.999391, 0.999229,
		 0.999048, 0.998848, 0.998630, 0.998392, 0.998135,
		 0.997859, 0.997564, 0.997250, 0.996917, 0.996566,
		 0.996195, 0.995805, 0.995396, 0.994969, 0.994522,
		 0.994056, 0.993572, 0.993069, 0.992546, 0.992005,
		 0.991445, 0.990866, 0.990268, 0.989651, 0.989016,
		 0.988362, 0.987688, 0.986996, 0.986286, 0.985556,
		 0.984808, 0.984041, 0.983255, 0.982451, 0.981627,
		 0.980785, 0.979925, 0.979046, 0.978148, 0.977231,
		 0.976296, 0.975342, 0.974370, 0.973379, 0.972370,
		 0.971342, 0.970296, 0.969231, 0.968148, 0.967046,
		 0.965926, 0.964787, 0.963631, 0.962455, 0.961262,
		 0.960050, 0.958820, 0.957572, 0.956305, 0.955020,
		 0.953717, 0.952396, 0.951057, 0.949699, 0.948324,
		 0.946930, 0.945519, 0.944089, 0.942642, 0.941176,
		 0.939693, 0.938192, 0.936672, 0.935135, 0.933581,
		 0.932008, 0.930418, 0.928810, 0.927184, 0.925541,
		 0.923880, 0.922201, 0.920505, 0.918791, 0.917060,
		 0.915312, 0.913546, 0.911762, 0.909962, 0.908143,
		 0.906308, 0.904455, 0.902586, 0.900699, 0.898794,
		 0.896873, 0.894935, 0.892979, 0.891007, 0.889017,
		 0.887011, 0.884988, 0.882948, 0.880891, 0.878817,
		 0.876727, 0.874620, 0.872496, 0.870356, 0.868199,
		 0.866026, 0.863836, 0.861630, 0.859407, 0.857168,
		 0.854912, 0.852641, 0.850353, 0.848048, 0.845728,
		 0.843392, 0.841039, 0.838671, 0.836287, 0.833886,
		 0.831470, 0.829038, 0.826590, 0.824127, 0.821647,
		 0.819152, 0.816642, 0.814116, 0.811574, 0.809017,
		 0.806445, 0.803857, 0.801254, 0.798636, 0.796002,
		 0.793354, 0.790690, 0.788011, 0.785317, 0.782609,
		 0.779885, 0.777146, 0.774393, 0.771625, 0.768842,
		 0.766045, 0.763233, 0.760406, 0.757566, 0.754710,
		 0.751840, 0.748956, 0.746058, 0.743145, 0.740219,
		 0.737278, 0.734323, 0.731354, 0.728372, 0.725375,
		 0.722365, 0.719340, 0.716303, 0.713251, 0.710186,
		 0.707107, 0.704015, 0.700910, 0.697791, 0.694659,
		 0.691514, 0.688355, 0.685184, 0.681999, 0.678801,
		 0.675591, 0.672367, 0.669131, 0.665882, 0.662621,
		 0.659346, 0.656060, 0.652760, 0.649449, 0.646125,
		 0.642788, 0.639440, 0.636079, 0.632706, 0.629321,
		 0.625924, 0.622515, 0.619095, 0.615662, 0.612218,
		 0.608762, 0.605295, 0.601816, 0.598325, 0.594824,
		 0.591310, 0.587786, 0.584250, 0.580704, 0.577146,
		 0.573577, 0.569998, 0.566407, 0.562806, 0.559194,
		 0.555571, 0.551938, 0.548294, 0.544640, 0.540975,
		 0.537300, 0.533615, 0.529920, 0.526215, 0.522499,
		 0.518774, 0.515039, 0.511294, 0.507539, 0.503775,
		 0.500001, 0.496217, 0.492424, 0.488622, 0.484810,
		 0.480990, 0.477160, 0.473321, 0.469472, 0.465615,
		 0.461749, 0.457875, 0.453991, 0.450099, 0.446199,
		 0.442290, 0.438372, 0.434446, 0.430512, 0.426570,
		 0.422619, 0.418661, 0.414694, 0.410720, 0.406738,
		 0.402748, 0.398750, 0.394745, 0.390732, 0.386712,
		 0.382684, 0.378650, 0.374608, 0.370558, 0.366502,
		 0.362439, 0.358369, 0.354292, 0.350208, 0.346118,
		 0.342021, 0.337918, 0.333808, 0.329692, 0.325569,
		 0.321440, 0.317306, 0.313165, 0.309018, 0.304865,
		 0.300707, 0.296543, 0.292373, 0.288197, 0.284016,
		 0.279830, 0.275638, 0.271441, 0.267239, 0.263032,
		 0.258820, 0.254603, 0.250381, 0.246154, 0.241923,
		 0.237687, 0.233446, 0.229201, 0.224952, 0.220698,
		 0.216441, 0.212179, 0.207913, 0.203643, 0.199369,
		 0.195091, 0.190810, 0.186525, 0.182237, 0.177945,
		 0.173649, 0.169351, 0.165049, 0.160744, 0.156436,
		 0.152124, 0.147810, 0.143494, 0.139174, 0.134852,
		 0.130527, 0.126200, 0.121870, 0.117538, 0.113204,
		 0.108868, 0.104530, 0.100189, 0.095847, 0.091503,
		 0.087157, 0.082809, 0.078460, 0.074110, 0.069758,
		 0.065404, 0.061050, 0.056694, 0.052337, 0.047979,
		 0.043620, 0.039261, 0.034901, 0.030540, 0.026178,
		 0.021816, 0.017454, 0.013091, 0.008728, 0.004364,
		 0.000001, -0.004362, -0.008725, -0.013088, -0.017451,
		 -0.021814, -0.026176, -0.030537, -0.034898, -0.039259,
		 -0.043618, -0.047977, -0.052335, -0.056692, -0.061047,
		 -0.065402, -0.069755, -0.074107, -0.078458, -0.082807,
		 -0.087155, -0.091500, -0.095845, -0.100187, -0.104527,
		 -0.108866, -0.113202, -0.117536, -0.121868, -0.126198,
		 -0.130525, -0.134850, -0.139172, -0.143491, -0.147808,
		 -0.152122, -0.156433, -0.160741, -0.165046, -0.169348,
		 -0.173647, -0.177942, -0.182234, -0.186523, -0.190808,
		 -0.195089, -0.199367, -0.203641, -0.207911, -0.212176,
		 -0.216438, -0.220696, -0.224950, -0.229199, -0.233444,
		 -0.237685, -0.241921, -0.246152, -0.250379, -0.254601,
		 -0.258818, -0.263030, -0.267237, -0.271439, -0.275636,
		 -0.279828, -0.284014, -0.288195, -0.292371, -0.296540,
		 -0.300705, -0.304863, -0.309016, -0.313163, -0.317303,
		 -0.321438, -0.325567, -0.329689, -0.333806, -0.337916,
		 -0.342019, -0.346116, -0.350206, -0.354290, -0.358367,
		 -0.362437, -0.366500, -0.370556, -0.374605, -0.378647,
		 -0.382682, -0.386710, -0.390730, -0.394743, -0.398748,
		 -0.402746, -0.406735, -0.410718, -0.414692, -0.418659,
		 -0.422617, -0.426568, -0.430510, -0.434444, -0.438370,
		 -0.442288, -0.446197, -0.450097, -0.453989, -0.457873,
		 -0.461747, -0.465613, -0.469470, -0.473319, -0.477158,
		 -0.480988, -0.484808, -0.488620, -0.492422, -0.496215,
		 -0.499999, -0.503773, -0.507537, -0.511292, -0.515037,
		 -0.518772, -0.522497, -0.526213, -0.529918, -0.533613,
		 -0.537298, -0.540973, -0.544638, -0.548292, -0.551936,
		 -0.555569, -0.559192, -0.562804, -0.566405, -0.569996,
		 -0.573575, -0.577144, -0.580702, -0.584249, -0.587784,
		 -0.591309, -0.594822, -0.598324, -0.601814, -0.605293,
		 -0.608760, -0.612216, -0.615660, -0.619093, -0.622514,
		 -0.625922, -0.629319, -0.632704, -0.636077, -0.639438,
		 -0.642787, -0.646123, -0.649447, -0.652759, -0.656058,
		 -0.659345, -0.662619, -0.665881, -0.669130, -0.672366,
		 -0.675589, -0.678800, -0.681997, -0.685182, -0.688354,
		 -0.691512, -0.694657, -0.697789, -0.700908, -0.704014,
		 -0.707106, -0.710184, -0.713249, -0.716301, -0.719339,
		 -0.722363, -0.725373, -0.728370, -0.731353, -0.734322,
		 -0.737276, -0.740217, -0.743144, -0.746056, -0.748955,
		 -0.751839, -0.754709, -0.757564, -0.760405, -0.763232,
		 -0.766044, -0.768841, -0.771624, -0.774392, -0.777145,
		 -0.779884, -0.782607, -0.785316, -0.788010, -0.790689,
		 -0.793352, -0.796001, -0.798635, -0.801253, -0.803856,
		 -0.806444, -0.809016, -0.811573, -0.814115, -0.816641,
		 -0.819151, -0.821646, -0.824125, -0.826589, -0.829037,
		 -0.831469, -0.833885, -0.836285, -0.838670, -0.841038,
		 -0.843391, -0.845727, -0.848047, -0.850351, -0.852639,
		 -0.854911, -0.857167, -0.859406, -0.861628, -0.863835,
		 -0.866025, -0.868198, -0.870355, -0.872495, -0.874619,
		 -0.876726, -0.878816, -0.880890, -0.882947, -0.884987,
		 -0.887010, -0.889016, -0.891006, -0.892978, -0.894934,
		 -0.896872, -0.898793, -0.900698, -0.902585, -0.904454,
		 -0.906307, -0.908143, -0.909961, -0.911761, -0.913545,
		 -0.915311, -0.917059, -0.918791, -0.920504, -0.922200,
		 -0.923879, -0.925540, -0.927183, -0.928809, -0.930417,
		 -0.932007, -0.933580, -0.935135, -0.936672, -0.938191,
		 -0.939692, -0.941175, -0.942641, -0.944089, -0.945518,
		 -0.946930, -0.948323, -0.949699, -0.951056, -0.952395,
		 -0.953716, -0.955019, -0.956304, -0.957571, -0.958819,
		 -0.960049, -0.961261, -0.962455, -0.963630, -0.964787,
		 -0.965925, -0.967046, -0.968147, -0.969231, -0.970295,
		 -0.971342, -0.972370, -0.973379, -0.974370, -0.975342,
		 -0.976296, -0.977231, -0.978147, -0.979045, -0.979924,
		 -0.980785, -0.981627, -0.982450, -0.983255, -0.984040,
		 -0.984807, -0.985556, -0.986285, -0.986996, -0.987688,
		 -0.988361, -0.989016, -0.989651, -0.990268, -0.990866,
		 -0.991445, -0.992005, -0.992546, -0.993068, -0.993572,
		 -0.994056, -0.994522, -0.994968, -0.995396, -0.995805,
		 -0.996195, -0.996565, -0.996917, -0.997250, -0.997564,
		 -0.997859, -0.998135, -0.998392, -0.998629, -0.998848,
		 -0.999048, -0.999229, -0.999391, -0.999534, -0.999657,
		 -0.999762, -0.999848, -0.999914, -0.999962, -0.999990,
		 -1.000000, -0.999990, -0.999962, -0.999914, -0.999848,
		 -0.999762, -0.999657, -0.999534, -0.999391, -0.999229,
		 -0.999048, -0.998848, -0.998630, -0.998392, -0.998135,
		 -0.997859, -0.997564, -0.997250, -0.996917, -0.996566,
		 -0.996195, -0.995805, -0.995396, -0.994969, -0.994522,
		 -0.994057, -0.993572, -0.993069, -0.992546, -0.992005,
		 -0.991445, -0.990866, -0.990268, -0.989652, -0.989016,
		 -0.988362, -0.987689, -0.986997, -0.986286, -0.985556,
		 -0.984808, -0.984041, -0.983255, -0.982451, -0.981628,
		 -0.980786, -0.979925, -0.979046, -0.978148, -0.977231,
		 -0.976296, -0.975343, -0.974370, -0.973380, -0.972370,
		 -0.971342, -0.970296, -0.969231, -0.968148, -0.967046,
		 -0.965926, -0.964788, -0.963631, -0.962456, -0.961262,
		 -0.960050, -0.958820, -0.957572, -0.956305, -0.955020,
		 -0.953717, -0.952396, -0.951057, -0.949700, -0.948324,
		 -0.946931, -0.945519, -0.944090, -0.942642, -0.941177,
		 -0.939693, -0.938192, -0.936673, -0.935136, -0.933581,
		 -0.932009, -0.930418, -0.928810, -0.927185, -0.925541,
		 -0.923880, -0.922202, -0.920506, -0.918792, -0.917061,
		 -0.915312, -0.913546, -0.911763, -0.909962, -0.908144,
		 -0.906309, -0.904456, -0.902586, -0.900699, -0.898795,
		 -0.896874, -0.894935, -0.892980, -0.891007, -0.889018,
		 -0.887012, -0.884989, -0.882948, -0.880892, -0.878818,
		 -0.876728, -0.874621, -0.872497, -0.870357, -0.868200,
		 -0.866026, -0.863836, -0.861630, -0.859407, -0.857168,
		 -0.854913, -0.852641, -0.850353, -0.848049, -0.845729,
		 -0.843392, -0.841040, -0.838672, -0.836287, -0.833887,
		 -0.831471, -0.829039, -0.826591, -0.824127, -0.821648,
		 -0.819153, -0.816643, -0.814117, -0.811575, -0.809018,
		 -0.806446, -0.803858, -0.801255, -0.798637, -0.796003,
		 -0.793355, -0.790691, -0.788012, -0.785318, -0.782609,
		 -0.779886, -0.777147, -0.774394, -0.771626, -0.768843,
		 -0.766046, -0.763234, -0.760407, -0.757566, -0.754711,
		 -0.751841, -0.748957, -0.746059, -0.743146, -0.740219,
		 -0.737279, -0.734324, -0.731355, -0.728372, -0.725376,
		 -0.722365, -0.719341, -0.716303, -0.713252, -0.710187,
		 -0.707108, -0.704016, -0.700911, -0.697792, -0.694660,
		 -0.691514, -0.688356, -0.685184, -0.682000, -0.678802,
		 -0.675592, -0.672368, -0.669132, -0.665883, -0.662622,
		 -0.659347, -0.656061, -0.652761, -0.649450, -0.646126,
		 -0.642789, -0.639441, -0.636080, -0.632707, -0.629322,
		 -0.625925, -0.622516, -0.619096, -0.615663, -0.612219,
		 -0.608763, -0.605296, -0.601817, -0.598326, -0.594824,
		 -0.591311, -0.587787, -0.584251, -0.580705, -0.577147,
		 -0.573578, -0.569998, -0.566408, -0.562807, -0.559195,
		 -0.555572, -0.551939, -0.548295, -0.544641, -0.540976,
		 -0.537301, -0.533616, -0.529921, -0.526216, -0.522500,
		 -0.518775, -0.515040, -0.511295, -0.507540, -0.503776,
		 -0.500002, -0.496218, -0.492425, -0.488623, -0.484811,
		 -0.480991, -0.477161, -0.473321, -0.469473, -0.465616,
		 -0.461750, -0.457876, -0.453992, -0.450100, -0.446200,
		 -0.442291, -0.438373, -0.434447, -0.430513, -0.426571,
		 -0.422620, -0.418662, -0.414695, -0.410721, -0.406739,
		 -0.402749, -0.398751, -0.394746, -0.390733, -0.386713,
		 -0.382685, -0.378651, -0.374609, -0.370559, -0.366503,
		 -0.362440, -0.358370, -0.354293, -0.350209, -0.346119,
		 -0.342022, -0.337919, -0.333809, -0.329693, -0.325570,
		 -0.321441, -0.317307, -0.313166, -0.309019, -0.304866,
		 -0.300708, -0.296544, -0.292374, -0.288198, -0.284017,
		 -0.279831, -0.275639, -0.271443, -0.267240, -0.263033,
		 -0.258821, -0.254604, -0.250382, -0.246155, -0.241924,
		 -0.237688, -0.233447, -0.229202, -0.224953, -0.220700,
		 -0.216442, -0.212180, -0.207914, -0.203644, -0.199370,
		 -0.195092, -0.190811, -0.186526, -0.182238, -0.177946,
		 -0.173650, -0.169352, -0.165050, -0.160745, -0.156437,
		 -0.152126, -0.147812, -0.143495, -0.139175, -0.134853,
		 -0.130528, -0.126201, -0.121872, -0.117540, -0.113205,
		 -0.108869, -0.104531, -0.100190, -0.095848, -0.091504,
		 -0.087158, -0.082810, -0.078461, -0.074111, -0.069759,
		 -0.065405, -0.061051, -0.056695, -0.052338, -0.047980,
		 -0.043622, -0.039262, -0.034902, -0.030541, -0.026179,
		 -0.021817, -0.017455, -0.013092, -0.008729, -0.004366,
};

const float cos_LUT[LUT_SIZE] =
{
		 1.000000, 0.999990, 0.999962, 0.999914, 0.999848,
		 0.999762, 0.999657, 0.999534, 0.999391, 0.999229,
		 0.999048, 0.998848, 0.998630, 0.998392, 0.998135,
		 0.997859, 0.997564, 0.997250, 0.996917, 0.996566,
		 0.996195, 0.995805, 0.995396, 0.994969, 0.994522,
		 0.994056, 0.993572, 0.993068, 0.992546, 0.992005,
		 0.991445, 0.990866, 0.990268, 0.989651, 0.989016,
		 0.988362, 0.987688, 0.986996, 0.986286, 0.985556,
		 0.984808, 0.984041, 0.983255, 0.982450, 0.981627,
		 0.980785, 0.979925, 0.979045, 0.978148, 0.977231,
		 0.976296, 0.975342, 0.974370, 0.973379, 0.972370,
		 0.971342, 0.970296, 0.969231, 0.968148, 0.967046,
		 0.965926, 0.964787, 0.963630, 0.962455, 0.961262,
		 0.960050, 0.958820, 0.957571, 0.956305, 0.955020,
		 0.953717, 0.952396, 0.951057, 0.949699, 0.948324,
		 0.946930, 0.945519, 0.944089, 0.942642, 0.941176,
		 0.939693, 0.938191, 0.936672, 0.935135, 0.933580,
		 0.932008, 0.930418, 0.928810, 0.927184, 0.925541,
		 0.923880, 0.922201, 0.920505, 0.918791, 0.917060,
		 0.915312, 0.913546, 0.911762, 0.909961, 0.908143,
		 0.906308, 0.904455, 0.902585, 0.900698, 0.898794,
		 0.896873, 0.894934, 0.892979, 0.891007, 0.889017,
		 0.887011, 0.884988, 0.882948, 0.880891, 0.878817,
		 0.876727, 0.874620, 0.872496, 0.870356, 0.868199,
		 0.866025, 0.863836, 0.861629, 0.859407, 0.857167,
		 0.854912, 0.852640, 0.850352, 0.848048, 0.845728,
		 0.843392, 0.841039, 0.838671, 0.836286, 0.833886,
		 0.831470, 0.829038, 0.826590, 0.824126, 0.821647,
		 0.819152, 0.816642, 0.814116, 0.811574, 0.809017,
		 0.806445, 0.803857, 0.801254, 0.798636, 0.796002,
		 0.793353, 0.790690, 0.788011, 0.785317, 0.782608,
		 0.779885, 0.777146, 0.774393, 0.771625, 0.768842,
		 0.766045, 0.763233, 0.760406, 0.757565, 0.754710,
		 0.751840, 0.748956, 0.746058, 0.743145, 0.740218,
		 0.737278, 0.734323, 0.731354, 0.728371, 0.725375,
		 0.722364, 0.719340, 0.716302, 0.713251, 0.710186,
		 0.707107, 0.704015, 0.700909, 0.697791, 0.694659,
		 0.691513, 0.688355, 0.685183, 0.681999, 0.678801,
		 0.675590, 0.672367, 0.669131, 0.665882, 0.662620,
		 0.659346, 0.656059, 0.652760, 0.649448, 0.646124,
		 0.642788, 0.639439, 0.636078, 0.632706, 0.629321,
		 0.625924, 0.622515, 0.619094, 0.615662, 0.612218,
		 0.608762, 0.605294, 0.601815, 0.598325, 0.594823,
		 0.591310, 0.587786, 0.584250, 0.580703, 0.577145,
		 0.573577, 0.569997, 0.566407, 0.562805, 0.559193,
		 0.555571, 0.551937, 0.548294, 0.544639, 0.540975,
		 0.537300, 0.533615, 0.529920, 0.526214, 0.522499,
		 0.518774, 0.515038, 0.511293, 0.507539, 0.503774,
		 0.500000, 0.496217, 0.492424, 0.488622, 0.484810,
		 0.480989, 0.477159, 0.473320, 0.469472, 0.465615,
		 0.461749, 0.457874, 0.453991, 0.450099, 0.446198,
		 0.442289, 0.438372, 0.434446, 0.430511, 0.426569,
		 0.422619, 0.418660, 0.414694, 0.410719, 0.406737,
		 0.402747, 0.398749, 0.394744, 0.390732, 0.386711,
		 0.382684, 0.378649, 0.374607, 0.370558, 0.366502,
		 0.362438, 0.358368, 0.354291, 0.350208, 0.346117,
		 0.342021, 0.337917, 0.333807, 0.329691, 0.325569,
		 0.321440, 0.317305, 0.313164, 0.309017, 0.304865,
		 0.300706, 0.296542, 0.292372, 0.288197, 0.284016,
		 0.279829, 0.275638, 0.271441, 0.267239, 0.263032,
		 0.258819, 0.254602, 0.250380, 0.246154, 0.241922,
		 0.237686, 0.233446, 0.229201, 0.224952, 0.220698,
		 0.216440, 0.212178, 0.207912, 0.203642, 0.199368,
		 0.195091, 0.190809, 0.186525, 0.182236, 0.177944,
		 0.173649, 0.169350, 0.165048, 0.160743, 0.156435,
		 0.152124, 0.147810, 0.143493, 0.139174, 0.134851,
		 0.130527, 0.126199, 0.121870, 0.117538, 0.113204,
		 0.108867, 0.104529, 0.100189, 0.095846, 0.091502,
		 0.087156, 0.082809, 0.078460, 0.074109, 0.069757,
		 0.065404, 0.061049, 0.056693, 0.052336, 0.047979,
		 0.043620, 0.039260, 0.034900, 0.030539, 0.026178,
		 0.021815, 0.017453, 0.013090, 0.008727, 0.004364,
		 0.000001, -0.004363, -0.008726, -0.013089, -0.017452,
		 -0.021814, -0.026176, -0.030538, -0.034899, -0.039259,
		 -0.043619, -0.047978, -0.052335, -0.056692, -0.061048,
		 -0.065403, -0.069756, -0.074108, -0.078459, -0.082808,
		 -0.087155, -0.091501, -0.095845, -0.100187, -0.104528,
		 -0.108866, -0.113203, -0.117537, -0.121869, -0.126198,
		 -0.130526, -0.134850, -0.139172, -0.143492, -0.147809,
		 -0.152123, -0.156434, -0.160742, -0.165047, -0.169349,
		 -0.173648, -0.177943, -0.182235, -0.186523, -0.190808,
		 -0.195090, -0.199367, -0.203641, -0.207911, -0.212177,
		 -0.216439, -0.220697, -0.224950, -0.229200, -0.233445,
		 -0.237685, -0.241921, -0.246153, -0.250379, -0.254601,
		 -0.258818, -0.263031, -0.267238, -0.271440, -0.275637,
		 -0.279828, -0.284015, -0.288196, -0.292371, -0.296541,
		 -0.300705, -0.304864, -0.309016, -0.313163, -0.317304,
		 -0.321439, -0.325568, -0.329690, -0.333806, -0.337916,
		 -0.342019, -0.346116, -0.350207, -0.354290, -0.358367,
		 -0.362437, -0.366501, -0.370557, -0.374606, -0.378648,
		 -0.382683, -0.386710, -0.390730, -0.394743, -0.398748,
		 -0.402746, -0.406736, -0.410718, -0.414693, -0.418659,
		 -0.422618, -0.426568, -0.430510, -0.434445, -0.438370,
		 -0.442288, -0.446197, -0.450098, -0.453990, -0.457873,
		 -0.461748, -0.465614, -0.469471, -0.473319, -0.477158,
		 -0.480988, -0.484809, -0.488621, -0.492423, -0.496216,
		 -0.499999, -0.503773, -0.507538, -0.511292, -0.515037,
		 -0.518773, -0.522498, -0.526213, -0.529919, -0.533614,
		 -0.537299, -0.540974, -0.544638, -0.548293, -0.551936,
		 -0.555570, -0.559192, -0.562804, -0.566406, -0.569996,
		 -0.573576, -0.577145, -0.580702, -0.584249, -0.587785,
		 -0.591309, -0.594822, -0.598324, -0.601814, -0.605293,
		 -0.608761, -0.612217, -0.615661, -0.619093, -0.622514,
		 -0.625923, -0.629320, -0.632705, -0.636078, -0.639438,
		 -0.642787, -0.646123, -0.649447, -0.652759, -0.656058,
		 -0.659345, -0.662619, -0.665881, -0.669130, -0.672366,
		 -0.675590, -0.678800, -0.681998, -0.685182, -0.688354,
		 -0.691512, -0.694658, -0.697790, -0.700909, -0.704014,
		 -0.707106, -0.710185, -0.713250, -0.716301, -0.719339,
		 -0.722363, -0.725374, -0.728370, -0.731353, -0.734322,
		 -0.737277, -0.740218, -0.743144, -0.746057, -0.748955,
		 -0.751839, -0.754709, -0.757564, -0.760405, -0.763232,
		 -0.766044, -0.768841, -0.771624, -0.774392, -0.777145,
		 -0.779884, -0.782608, -0.785316, -0.788010, -0.790689,
		 -0.793353, -0.796001, -0.798635, -0.801253, -0.803856,
		 -0.806444, -0.809016, -0.811573, -0.814115, -0.816641,
		 -0.819152, -0.821646, -0.824126, -0.826589, -0.829037,
		 -0.831469, -0.833885, -0.836286, -0.838670, -0.841039,
		 -0.843391, -0.845727, -0.848048, -0.850352, -0.852640,
		 -0.854911, -0.857167, -0.859406, -0.861629, -0.863835,
		 -0.866025, -0.868198, -0.870355, -0.872496, -0.874619,
		 -0.876726, -0.878817, -0.880890, -0.882947, -0.884987,
		 -0.887010, -0.889017, -0.891006, -0.892979, -0.894934,
		 -0.896872, -0.898794, -0.900698, -0.902585, -0.904455,
		 -0.906307, -0.908143, -0.909961, -0.911762, -0.913545,
		 -0.915311, -0.917060, -0.918791, -0.920504, -0.922201,
		 -0.923879, -0.925540, -0.927183, -0.928809, -0.930417,
		 -0.932008, -0.933580, -0.935135, -0.936672, -0.938191,
		 -0.939692, -0.941176, -0.942641, -0.944089, -0.945518,
		 -0.946930, -0.948323, -0.949699, -0.951056, -0.952395,
		 -0.953717, -0.955020, -0.956304, -0.957571, -0.958819,
		 -0.960050, -0.961261, -0.962455, -0.963630, -0.964787,
		 -0.965926, -0.967046, -0.968147, -0.969231, -0.970295,
		 -0.971342, -0.972370, -0.973379, -0.974370, -0.975342,
		 -0.976296, -0.977231, -0.978147, -0.979045, -0.979924,
		 -0.980785, -0.981627, -0.982450, -0.983255, -0.984041,
		 -0.984808, -0.985556, -0.986285, -0.986996, -0.987688,
		 -0.988361, -0.989016, -0.989651, -0.990268, -0.990866,
		 -0.991445, -0.992005, -0.992546, -0.993068, -0.993572,
		 -0.994056, -0.994522, -0.994968, -0.995396, -0.995805,
		 -0.996195, -0.996565, -0.996917, -0.997250, -0.997564,
		 -0.997859, -0.998135, -0.998392, -0.998629, -0.998848,
		 -0.999048, -0.999229, -0.999391, -0.999534, -0.999657,
		 -0.999762, -0.999848, -0.999914, -0.999962, -0.999990,
		 -1.000000, -0.999990, -0.999962, -0.999914, -0.999848,
		 -0.999762, -0.999657, -0.999534, -0.999391, -0.999229,
		 -0.999048, -0.998848, -0.998630, -0.998392, -0.998135,
		 -0.997859, -0.997564, -0.997250, -0.996917, -0.996566,
		 -0.996195, -0.995805, -0.995396, -0.994969, -0.994522,
		 -0.994056, -0.993572, -0.993069, -0.992546, -0.992005,
		 -0.991445, -0.990866, -0.990268, -0.989652, -0.989016,
		 -0.988362, -0.987689, -0.986997, -0.986286, -0.985556,
		 -0.984808, -0.984041, -0.983255, -0.982451, -0.981627,
		 -0.980786, -0.979925, -0.979046, -0.978148, -0.977231,
		 -0.976296, -0.975343, -0.974370, -0.973380, -0.972370,
		 -0.971342, -0.970296, -0.969231, -0.968148, -0.967046,
		 -0.965926, -0.964788, -0.963631, -0.962456, -0.961262,
		 -0.960050, -0.958820, -0.957572, -0.956305, -0.955020,
		 -0.953717, -0.952396, -0.951057, -0.949700, -0.948324,
		 -0.946931, -0.945519, -0.944089, -0.942642, -0.941176,
		 -0.939693, -0.938192, -0.936673, -0.935136, -0.933581,
		 -0.932008, -0.930418, -0.928810, -0.927184, -0.925541,
		 -0.923880, -0.922201, -0.920505, -0.918792, -0.917061,
		 -0.915312, -0.913546, -0.911763, -0.909962, -0.908144,
		 -0.906308, -0.904456, -0.902586, -0.900699, -0.898795,
		 -0.896873, -0.894935, -0.892980, -0.891007, -0.889018,
		 -0.887011, -0.884988, -0.882948, -0.880891, -0.878818,
		 -0.876727, -0.874620, -0.872497, -0.870356, -0.868199,
		 -0.866026, -0.863836, -0.861630, -0.859407, -0.857168,
		 -0.854913, -0.852641, -0.850353, -0.848049, -0.845729,
		 -0.843392, -0.841040, -0.838671, -0.836287, -0.833887,
		 -0.831470, -0.829038, -0.826591, -0.824127, -0.821648,
		 -0.819153, -0.816642, -0.814116, -0.811575, -0.809018,
		 -0.806445, -0.803858, -0.801255, -0.798636, -0.796003,
		 -0.793354, -0.790690, -0.788012, -0.785318, -0.782609,
		 -0.779885, -0.777147, -0.774394, -0.771625, -0.768843,
		 -0.766045, -0.763233, -0.760407, -0.757566, -0.754710,
		 -0.751841, -0.748957, -0.746058, -0.743146, -0.740219,
		 -0.737278, -0.734323, -0.731355, -0.728372, -0.725375,
		 -0.722365, -0.719341, -0.716303, -0.713251, -0.710186,
		 -0.707108, -0.704016, -0.700910, -0.697791, -0.694659,
		 -0.691514, -0.688356, -0.685184, -0.681999, -0.678802,
		 -0.675591, -0.672368, -0.669132, -0.665883, -0.662621,
		 -0.659347, -0.656060, -0.652761, -0.649449, -0.646125,
		 -0.642789, -0.639440, -0.636079, -0.632706, -0.629322,
		 -0.625925, -0.622516, -0.619095, -0.615663, -0.612218,
		 -0.608763, -0.605295, -0.601816, -0.598326, -0.594824,
		 -0.591311, -0.587786, -0.584251, -0.580704, -0.577146,
		 -0.573578, -0.569998, -0.566407, -0.562806, -0.559194,
		 -0.555571, -0.551938, -0.548294, -0.544640, -0.540976,
		 -0.537301, -0.533616, -0.529921, -0.526215, -0.522500,
		 -0.518775, -0.515039, -0.511294, -0.507540, -0.503775,
		 -0.500001, -0.496218, -0.492425, -0.488623, -0.484811,
		 -0.480990, -0.477160, -0.473321, -0.469473, -0.465616,
		 -0.461750, -0.457875, -0.453992, -0.450100, -0.446199,
		 -0.442290, -0.438373, -0.434447, -0.430512, -0.426570,
		 -0.422620, -0.418661, -0.414695, -0.410720, -0.406738,
		 -0.402748, -0.398750, -0.394745, -0.390733, -0.386712,
		 -0.382685, -0.378650, -0.374608, -0.370559, -0.366503,
		 -0.362439, -0.358369, -0.354292, -0.350209, -0.346119,
		 -0.342022, -0.337918, -0.333808, -0.329692, -0.325570,
		 -0.321441, -0.317306, -0.313165, -0.309018, -0.304866,
		 -0.300707, -0.296543, -0.292373, -0.288198, -0.284017,
		 -0.279831, -0.275639, -0.271442, -0.267240, -0.263033,
		 -0.258821, -0.254603, -0.250382, -0.246155, -0.241923,
		 -0.237687, -0.233447, -0.229202, -0.224953, -0.220699,
		 -0.216441, -0.212179, -0.207913, -0.203643, -0.199370,
		 -0.195092, -0.190811, -0.186526, -0.182237, -0.177945,
		 -0.173650, -0.169351, -0.165049, -0.160744, -0.156436,
		 -0.152125, -0.147811, -0.143494, -0.139175, -0.134853,
		 -0.130528, -0.126201, -0.121871, -0.117539, -0.113205,
		 -0.108869, -0.104530, -0.100190, -0.095847, -0.091503,
		 -0.087157, -0.082810, -0.078461, -0.074110, -0.069758,
		 -0.065405, -0.061050, -0.056694, -0.052338, -0.047980,
		 -0.043621, -0.039261, -0.034901, -0.030540, -0.026179,
		 -0.021817, -0.017454, -0.013091, -0.008728, -0.004365,
		 -0.000002, 0.004362, 0.008725, 0.013088, 0.017451,
		 0.021813, 0.026175, 0.030537, 0.034898, 0.039258,
		 0.043618, 0.047976, 0.052334, 0.056691, 0.061047,
		 0.065401, 0.069755, 0.074107, 0.078457, 0.082806,
		 0.087154, 0.091500, 0.095844, 0.100186, 0.104527,
		 0.108865, 0.113201, 0.117536, 0.121868, 0.126197,
		 0.130524, 0.134849, 0.139171, 0.143491, 0.147808,
		 0.152122, 0.156433, 0.160741, 0.165046, 0.169348,
		 0.173646, 0.177942, 0.182234, 0.186522, 0.190807,
		 0.195089, 0.199366, 0.203640, 0.207910, 0.212176,
		 0.216438, 0.220696, 0.224949, 0.229199, 0.233444,
		 0.237684, 0.241920, 0.246152, 0.250378, 0.254600,
		 0.258817, 0.263029, 0.267237, 0.271439, 0.275636,
		 0.279827, 0.284014, 0.288195, 0.292370, 0.296540,
		 0.300704, 0.304863, 0.309015, 0.313162, 0.317303,
		 0.321438, 0.325566, 0.329689, 0.333805, 0.337915,
		 0.342018, 0.346115, 0.350206, 0.354289, 0.358366,
		 0.362436, 0.366500, 0.370556, 0.374605, 0.378647,
		 0.382682, 0.386709, 0.390729, 0.394742, 0.398747,
		 0.402745, 0.406735, 0.410717, 0.414692, 0.418658,
		 0.422617, 0.426567, 0.430509, 0.434444, 0.438369,
		 0.442287, 0.446196, 0.450097, 0.453989, 0.457872,
		 0.461747, 0.465613, 0.469470, 0.473318, 0.477157,
		 0.480987, 0.484808, 0.488620, 0.492422, 0.496215,
		 0.499998, 0.503772, 0.507537, 0.511291, 0.515036,
		 0.518772, 0.522497, 0.526212, 0.529918, 0.533613,
		 0.537298, 0.540973, 0.544637, 0.548292, 0.551935,
		 0.555569, 0.559191, 0.562803, 0.566405, 0.569995,
		 0.573575, 0.577144, 0.580701, 0.584248, 0.587784,
		 0.591308, 0.594821, 0.598323, 0.601813, 0.605292,
		 0.608760, 0.612216, 0.615660, 0.619092, 0.622513,
		 0.625922, 0.629319, 0.632704, 0.636077, 0.639438,
		 0.642786, 0.646123, 0.649447, 0.652758, 0.656058,
		 0.659344, 0.662619, 0.665880, 0.669129, 0.672365,
		 0.675589, 0.678799, 0.681997, 0.685182, 0.688353,
		 0.691512, 0.694657, 0.697789, 0.700908, 0.704013,
		 0.707105, 0.710184, 0.713249, 0.716301, 0.719338,
		 0.722363, 0.725373, 0.728370, 0.731352, 0.734321,
		 0.737276, 0.740217, 0.743143, 0.746056, 0.748954,
		 0.751838, 0.754708, 0.757564, 0.760405, 0.763231,
		 0.766043, 0.768841, 0.771623, 0.774391, 0.777145,
		 0.779883, 0.782607, 0.785316, 0.788010, 0.790688,
		 0.793352, 0.796001, 0.798634, 0.801253, 0.803856,
		 0.806443, 0.809016, 0.811573, 0.814114, 0.816640,
		 0.819151, 0.821646, 0.824125, 0.826589, 0.829036,
		 0.831468, 0.833885, 0.836285, 0.838669, 0.841038,
		 0.843390, 0.845727, 0.848047, 0.850351, 0.852639,
		 0.854911, 0.857166, 0.859405, 0.861628, 0.863834,
		 0.866024, 0.868198, 0.870355, 0.872495, 0.874619,
		 0.876726, 0.878816, 0.880890, 0.882947, 0.884987,
		 0.887010, 0.889016, 0.891006, 0.892978, 0.894933,
		 0.896872, 0.898793, 0.900697, 0.902584, 0.904454,
		 0.906307, 0.908142, 0.909960, 0.911761, 0.913545,
		 0.915311, 0.917059, 0.918790, 0.920504, 0.922200,
		 0.923879, 0.925540, 0.927183, 0.928809, 0.930417,
		 0.932007, 0.933580, 0.935134, 0.936671, 0.938191,
		 0.939692, 0.941175, 0.942641, 0.944088, 0.945518,
		 0.946929, 0.948323, 0.949698, 0.951056, 0.952395,
		 0.953716, 0.955019, 0.956304, 0.957571, 0.958819,
		 0.960049, 0.961261, 0.962455, 0.963630, 0.964787,
		 0.965925, 0.967045, 0.968147, 0.969230, 0.970295,
		 0.971342, 0.972369, 0.973379, 0.974370, 0.975342,
		 0.976296, 0.977231, 0.978147, 0.979045, 0.979924,
		 0.980785, 0.981627, 0.982450, 0.983255, 0.984040,
		 0.984807, 0.985556, 0.986285, 0.986996, 0.987688,
		 0.988361, 0.989016, 0.989651, 0.990268, 0.990866,
		 0.991445, 0.992005, 0.992546, 0.993068, 0.993572,
		 0.994056, 0.994522, 0.994968, 0.995396, 0.995805,
		 0.996195, 0.996565, 0.996917, 0.997250, 0.997564,
		 0.997859, 0.998135, 0.998392, 0.998629, 0.998848,
		 0.999048, 0.999229, 0.999391, 0.999534, 0.999657,
		 0.999762, 0.999848, 0.999914, 0.999962, 0.999990,
};

