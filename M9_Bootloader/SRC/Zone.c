#include <stdlib.h>
#include "CorMove.h"
#include "Map.h"
#include "PathPlanning.h"
#include "ShortestPath.h"
#include "WallFollowMulti.h"
#include "Zone.h"

#ifdef ZONE_WALLFOLLOW
//Normal zone
static uint8_t zones_cnt = 0;
volatile ZoneWallFollowType zones[ZONE_MAX];
static Point16_t uppestCell, downestCell, rightestCell, leftestCell;


static uint8_t continualZones[ZONE_MAX] = {0};
static uint8_t continualZonesSize = 0;
static uint8_t isContinual = 1;


//Trapped Zone
static uint8_t trappedZoneIdx[10];
static uint8_t trappedZoneSize = 0;

//Finish zone wall follow
//0: All map cleaning;
//1: Finish cleaning / Middle zone cleaning;
//2: Normal zone cleaning
//3: Trapped mode, find path to trapped zone
static uint8_t zoneWallFollowStatus;

static uint8_t tooNearZoneCnt = 0;

static CellState boundary_cell[4][ZONE_SIZE + 2 * BOUNDARY_INCREMENT + 1];

extern int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

extern volatile uint16_t path_blocks_size;

/**
 * Initialize
 */
void Zone_Initialize() {
	uint8_t i = 0;

	path_blocks_size = 0;
	
	zones_cnt = 0;
	for ( i = 0; i < ZONE_MAX; ++i ) {
		zones[i].zone.X = 0;
		zones[i].zone.Y = 0;
		zones[i].entranceCell.X = 0;
		zones[i].entranceCell.Y = 0;
		zones[i].exitCell.X = 0;
		zones[i].exitCell.Y = 0;
		zones[i].entranceHeading = 0;
		zones[i].exitHeading = 0;

		continualZones[i] = 0;
	}

	uppestCell.X		= 0; uppestCell.Y	= 0;
	downestCell.X	= 0; downestCell.Y	= 0;
	rightestCell.X	= 0; rightestCell.Y	= 0;
	leftestCell.X	= 0; leftestCell.Y	= 0;

	continualZonesSize = 0;
	isContinual = 1;

	for ( i = 0; i < 10; ++i ) {
		trappedZoneIdx[i] = 0;
	}
	trappedZoneSize = 0;

	zoneWallFollowStatus = 0;
}

/**
 * Check if there is a new zone
 * @param	start_x Start Cell x
 * @param	start_y Start Cell Y
 * @param	des_x   End Cell x
 * @param	des_y   End Cell y
 * @return         1: successfully find; 0 or others: fail
 */
int8_t Zone_IsAddNewZone( int16_t start_x, int16_t start_y,
                          int16_t des_x, int16_t des_y ) {
	int8_t retval = 0/*, i = 0*/;
	uint8_t gap = ROBOT_SIZE / 2 + 1;
	uint8_t increment = ROBOT_SIZE + 1;
											
	if ( zones_cnt == 0 ) {
		increment = ( ROBOT_SIZE + 1 ) * 2;
	}

	//Renew the upest, downest, rightest and leftest point
	if ( des_x > uppestCell.X ) {
		uppestCell.X = des_x;
		uppestCell.Y = des_y;
	}

	if ( des_x < downestCell.X ) {
		downestCell.X = des_x;
		downestCell.Y = des_y;
	}

	if ( des_y < rightestCell.Y ) {
		rightestCell.X = des_x;
		rightestCell.Y = des_y;
	}

	if ( des_y > leftestCell.Y ) {
		leftestCell.X = des_x;
		leftestCell.Y = des_y;
	}

	if ( uppestCell.X - downestCell.X > ZONE_SIZE - 1 - increment ||
			 leftestCell.Y - rightestCell.Y > ZONE_SIZE - 1 - increment ) {
		USPRINTF("Increment: %d\n", increment);
		#ifdef RIGHT_WALL
		//Case 1: >	-45, < 45
		if ( course2dest( start_x, start_y, des_x, des_y ) < 450 ||
				 course2dest( start_x, start_y, des_x, des_y ) >= 3150 ) {

			retval = Zone_AddZone( start_x, start_y, des_x, des_y, uppestCell.X + gap - ZONE_SIZE_HALF,
			                                                       rightestCell.Y - gap + ZONE_SIZE_HALF );
			USPRINTF("Zone Direction: -45~45");
		}
		//Case 2: >	 45, < 135
		else if ( course2dest( start_x, start_y, des_x, des_y ) >= 450 &&
		          course2dest( start_x, start_y, des_x, des_y ) < 1350 ) {

			retval = Zone_AddZone( start_x, start_y, des_x, des_y, uppestCell.X + gap - ZONE_SIZE_HALF,
			                                                       leftestCell.Y + gap - ZONE_SIZE_HALF );
			USPRINTF("Zone Direction: 45~135");
		}
		//Case 3: >	135, < -135
		else if ( course2dest( start_x, start_y, des_x, des_y ) < 2250 &&
		          course2dest( start_x, start_y, des_x, des_y ) >= 1350 ) {

			retval = Zone_AddZone( start_x, start_y, des_x, des_y, downestCell.X - gap + ZONE_SIZE_HALF,
			                                                       leftestCell.Y + gap - ZONE_SIZE_HALF );
			USPRINTF("Zone Direction: 135~225");
		}
		//Case 4: > -135, < -45
		else if ( course2dest( start_x, start_y, des_x, des_y ) >= 2250 &&
		          course2dest( start_x, start_y, des_x, des_y ) < 3150 ) {

			retval = Zone_AddZone( start_x, start_y, des_x, des_y, downestCell.X - gap + ZONE_SIZE_HALF,
			                                                       rightestCell.Y - gap + ZONE_SIZE_HALF );
			USPRINTF("Zone Direction: 225~315");
		}
		#endif
		
		if ( retval == -1 || retval == -2 ) {
			USPRINTF("Cannot add zone! Go home!\n");
			CM_SetGoHome(0);
			return 1;
		}

#ifdef STOP_WALL_FOLLOW_M1
		
		if((CM_IsFromStation()==0)&&(zones_cnt>2))
		{
			USPRINTF("Current Zone Entrance: (%d, %d), Exit: (%d, %d)\n", zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																																		zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y);
			USPRINTF("Zone 0 Entrance: (%d, %d), Exit: (%d, %d)\n",
							 zones[continualZones[0]].entranceCell.X, zones[continualZones[0]].entranceCell.Y,
							 zones[continualZones[0]].exitCell.X, zones[continualZones[0]].exitCell.Y );
			USPRINTF("Distance to First Zone Entrance: %d\n", TwoPointsDistance( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																																					 zones[continualZones[0]].entranceCell.X, zones[continualZones[0]].entranceCell.Y) );
			USPRINTF("Distance to First Zone Exit: %d\n", TwoPointsDistance( zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y,
																																			 zones[continualZones[0]].exitCell.X, zones[continualZones[0]].exitCell.Y) );
			USPRINTF("Delta Angle between Two Zone: %d\n", angle_delta( course2dest( zones[continualZones[0]].entranceCell.X, zones[continualZones[0]].entranceCell.Y,
																																							 zones[continualZones[0]].exitCell.X, zones[continualZones[0]].exitCell.Y ),
																																	course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																																							 zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) );

			//Case 1, only check first zone
			if ( TwoPointsDistance( zones[continualZones[0]].entranceCell.X, zones[continualZones[0]].entranceCell.Y,
															zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y ) < STOP_WALL_FOLLOW_M1_MAX_DISTANCE &&
					 TwoPointsDistance( zones[continualZones[0]].exitCell.X, zones[continualZones[0]].exitCell.Y,
															zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) < STOP_WALL_FOLLOW_M1_MAX_DISTANCE &&
					 angle_delta( course2dest( zones[continualZones[0]].entranceCell.X, zones[continualZones[0]].entranceCell.Y,
																		 zones[continualZones[0]].exitCell.X,     zones[continualZones[0]].exitCell.Y ),
												course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																		 zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) < 450 &&
					 continualZonesSize > STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE ) {
				USPRINTF("%s %d Method 1 Case 1 to stop Wall Follow!\n", __FUNCTION__, __LINE__);
				Zone_SetZoneWallFollowStatus(1);
				return 1;
			}
		

			//Case 2, check other zones
			for ( i = 1; continualZonesSize > STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE &&
									 ( continualZones[i] < zones_cnt - STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE ) &&
									 i < continualZonesSize - STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE + 1 &&
									 i < STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE; ++i ) {

				//USPRINTF("Current Zone Entrance: (%d, %d), Exit: (%d, %d)\n", start_x, start_y, des_x, des_y);
				USPRINTF("Zone %d Entrance: (%d, %d), Exit: (%d, %d)\n", continualZones[i],
								 zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
								 zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y );
				USPRINTF("Distance to Zone %d Entrance: %d\n", continualZones[i], TwoPointsDistance( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																																														 zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y) );
				USPRINTF("Distance to Zone %d Exit: %d\n", continualZones[i], TwoPointsDistance( zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y,
																																												 zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y) );
				USPRINTF("Delta Angle between Two Zone: %d\n", angle_delta( course2dest( zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
																																								 zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y ),
																																		course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																																								 zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) );

				if ( TwoPointsDistance( zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
																zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y ) < STOP_WALL_FOLLOW_M1_MAX_DISTANCE &&
						 TwoPointsDistance( zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y,
																zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) < STOP_WALL_FOLLOW_M1_MAX_DISTANCE &&
						 angle_delta( course2dest( zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
																			 zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y ),
													course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
																			 zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) < 450 ) {
					USPRINTF("%s %d Method 1 Case 2 to stop Wall Follow!\n", __FUNCTION__, __LINE__);
					Zone_SetZoneWallFollowStatus(1);
					return 1;
				}
			}
		}
		//Check if zone is too near
		isTooNearZoneFind = 0;
		USPRINTF("Too near zone count check...\n");
		for ( i = 0; continualZonesSize > STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE &&
		             ( continualZones[i] < zones_cnt - STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE ) &&
		             i < continualZonesSize - STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE + 1 &&
		             i < STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE; ++i ) {

			USPRINTF("Current Zone Zone: (%d, %d)\n", zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y);
			USPRINTF("Zone %d Zone: (%d, %d)\n", continualZones[i], zones[continualZones[i]].zone.X, zones[continualZones[i]].zone.Y );
			USPRINTF("Distance to Zone %d Entrance: %d\n", continualZones[i], TwoPointsDistance(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y,
			                                                                                    zones[continualZones[i]].zone.X, zones[continualZones[i]].zone.Y ) );

			if ( TwoPointsDistance(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y,
			                       zones[continualZones[i]].zone.X, zones[continualZones[i]].zone.Y ) < ZONE_SIZE / 2 ) {
				tooNearZoneCnt++;
				isTooNearZoneFind = 1;
				break;
			}
		}

		
		if ( isTooNearZoneFind == 0 ) {
			tooNearZoneCnt = 0;
		} else if ( tooNearZoneCnt > 5 ) {
			USPRINTF("Too near zone too much!Stop wall follow!\n");
//			Zone_SetZoneWallFollowStatus(1);
		}

		USPRINTF("Too near zone count: %d\n", tooNearZoneCnt);
#endif

#ifdef STOP_WALL_FOLLOW_M2
		if ( cnt > ZONE_SIZE_HALF ) {
			for ( i = 0; i < cnt / 2 - ZONE_SIZE_HALF ; ++i ) {
				intersectPnt = calIntersPoint( wallFollowCellPtr[i], wallFollowCellPtr[i + 1],
				                               wallFollowCellPtr[cnt - 2], wallFollowCellPtr[cnt - 1] );

				if ( TwoPointsDistance( wallFollowCellPtr[i].X, wallFollowCellPtr[i].Y, wallFollowCellPtr[cnt - 2].X, wallFollowCellPtr[cnt - 2].Y ) < 6 ||
				     //TwoPointsDistance( wallFollowCellPtr[i].X, wallFollowCellPtr[i].Y, wallFollowCellPtr[cnt - 1].X, wallFollowCellPtr[cnt - 1].Y ) <= 6 ||
				     //TwoPointsDistance( wallFollowCellPtr[i+1].X, wallFollowCellPtr[i+1].Y, wallFollowCellPtr[cnt - 2].X, wallFollowCellPtr[cnt - 2].Y ) <= 6 ||
				     TwoPointsDistance( wallFollowCellPtr[i+1].X, wallFollowCellPtr[i+1].Y, wallFollowCellPtr[cnt - 1].X, wallFollowCellPtr[cnt - 1].Y ) < 6 ) {
					USPRINTF("%s %d Debug: Cross Line1 Points: 1: x:%d, y:%d\t2: x: %d, y: %d\n",
					         __FUNCTION__, __LINE__, wallFollowCellPtr[i].X, wallFollowCellPtr[i].Y, wallFollowCellPtr[i + 1].X, wallFollowCellPtr[i + 1].Y);
					USPRINTF("%s %d Debug: Cross Line2 Points: 2: x:%d, y:%d\t2: x: %d, y: %d\n",
					         __FUNCTION__, __LINE__, wallFollowCellPtr[cnt - 2].X, wallFollowCellPtr[cnt - 2].Y, wallFollowCellPtr[cnt - 1].X, wallFollowCellPtr[cnt - 1].Y);
					USPRINTF("%s %d Debug: Intersection Point: x:%d, y:%d\n", __FUNCTION__, __LINE__, intersectPnt.X, intersectPnt.Y);
				}

				//Zero point check that if the intersection point is in these two line segment
				//f(x1) * f(x2) < 0, then means that there is solution between x1 and x2
				if ( (intersectPnt.X - wallFollowCellPtr[i].X) * (intersectPnt.X - wallFollowCellPtr[i + 1].X) <= 0 &&
				     (intersectPnt.Y - wallFollowCellPtr[i].Y) * (intersectPnt.Y - wallFollowCellPtr[i + 1].Y) <= 0 &&
				     (intersectPnt.X - wallFollowCellPtr[cnt - 2].X) * (intersectPnt.X - wallFollowCellPtr[cnt - 1].X) <= 0 &&
				     (intersectPnt.Y - wallFollowCellPtr[cnt - 2].Y) * (intersectPnt.Y - wallFollowCellPtr[cnt - 1].Y) <= 0
				   ) {
					USPRINTF("%s %d Wall Follow Idx: %d\tCnt: %d\n", __FUNCTION__, __LINE__, i, cnt);

					USPRINTF("%s %d Wall Follow Cross Line1 Points: 1: x:%d, y:%d\t2: x: %d, y: %d\n",
					         __FUNCTION__, __LINE__, wallFollowCellPtr[i].X, wallFollowCellPtr[i].Y, wallFollowCellPtr[i + 1].X, wallFollowCellPtr[i + 1].Y);
					USPRINTF("%s %d Wall Follow Cross Line2 Points: 2: x:%d, y:%d\t2: x: %d, y: %d\n",
					         __FUNCTION__, __LINE__, wallFollowCellPtr[cnt - 2].X, wallFollowCellPtr[cnt - 2].Y, wallFollowCellPtr[cnt - 1].X, wallFollowCellPtr[cnt - 1].Y);
					USPRINTF("%s %d Wall Follow Lines Intersection Point: x:%d, y:%d\n", __FUNCTION__, __LINE__, intersectPnt.X, intersectPnt.Y);
					USPRINTF("%s %d Method 2 to stop Wall Follow!\n", __FUNCTION__, __LINE__);
					Zone_SetZoneWallFollowStatus(1);
					return 1;
				}
			}
		}
#endif

#ifdef STOP_WALL_FOLLOW_M3
		if((CM_IsFromStation()==0)&&(zones_cnt>2))
		{
			USPRINTF("Method 3 Checking...\n");
			if ( zones_cnt > 1 ) {
					for ( i = 0; i < (continualZonesSize / 2 > STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE ? STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE : continualZonesSize / 2) &&
						continualZones[i] < zones_cnt - STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE &&
						i < continualZonesSize - STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE + 1; ++i ) {
					intersectPnt = calIntersPoint( zones[continualZones[i]].entranceCell, zones[continualZones[i]].exitCell,
																				 zones[zones_cnt - 1].entranceCell, zones[zones_cnt - 1].exitCell );

					USPRINTF("Zone %d cross Line1 points: Entrance: x:%d, y:%d\tExit: x: %d, y: %d\n", continualZones[i],
									 zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
									 zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y);
					USPRINTF("Current zone cross Line2 points: Entrance: x:%d, y:%d\tExit: x: %d, y: %d\n",
									 zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
									 zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y);
					USPRINTF("Intersection point: x:%d, y:%d\n", intersectPnt.X, intersectPnt.Y);

					//Use zero point method to check if the intersection point is within these two line segment
					//f(x1) * f(x2) < 0, then means that there is solution between x1 and x2
					if ( (intersectPnt.X - zones[continualZones[i]].entranceCell.X) *
							 (intersectPnt.X - zones[continualZones[i]].exitCell.X) <= 0 &&
							 (intersectPnt.Y - zones[continualZones[i]].entranceCell.Y) *
							 (intersectPnt.Y - zones[continualZones[i]].exitCell.Y) <= 0 &&
							 (intersectPnt.X - zones[zones_cnt - 1].entranceCell.X) *
							 (intersectPnt.X - zones[zones_cnt - 1].exitCell.X) <= 0 &&
							 (intersectPnt.Y - zones[zones_cnt - 1].entranceCell.Y) *
							 (intersectPnt.Y - zones[zones_cnt - 1].exitCell.Y) <= 0
						 ) {

						USPRINTF("Delta Angle between Two Zone: %d\n",
							angle_delta( course2dest( zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
							zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y ),
							course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
							zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) );


						if (angle_delta( course2dest( zones[continualZones[i]].entranceCell.X, zones[continualZones[i]].entranceCell.Y,
										zones[continualZones[i]].exitCell.X, zones[continualZones[i]].exitCell.Y ),
								course2dest( zones[zones_cnt - 1].entranceCell.X, zones[zones_cnt - 1].entranceCell.Y,
										zones[zones_cnt - 1].exitCell.X, zones[zones_cnt - 1].exitCell.Y ) ) < 450 ) {
							USPRINTF("Method 3 to stop wall follow!\n");
							Zone_SetZoneWallFollowStatus(1);
							return 1;
						}
					}
				}
			}
		}
#endif

	} else {
		retval = 0;
	}
	return retval;
}

int8_t Zone_ForceAddZone( int16_t start_x, int16_t start_y, int16_t des_x, int16_t des_y ) {
	int8_t retval = 0;
	retval = Zone_AddZone( start_x, start_y, des_x, des_y, ( uppestCell.X + downestCell.X ) / 2,
	                                                       ( leftestCell.Y + rightestCell.Y ) / 2 );

#if 0
	uint8_t gap = ROBOT_SIZE / 2 + 1;
	//Case 1: >  -45, < 45
	if ( Zone_GetLastZoneDirection() < 450 || Zone_GetLastZoneDirection() >= 3150 ) {
		retval = Zone_AddZone( start_x, start_y, des_x, des_y, downestCell.X - gap + ZONE_SIZE_HALF,
		                                                       leftestCell.Y + gap - ZONE_SIZE_HALF );
		USPRINTF("Zone Direction: -45~45");
	}
	//Case 2: >   45, < 135
	else if ( Zone_GetLastZoneDirection() >= 450 && Zone_GetLastZoneDirection() < 1350 ) {
		retval = Zone_AddZone( start_x, start_y, des_x, des_y, downestCell.X - gap + ZONE_SIZE_HALF,
		                                                       rightestCell.Y - gap + ZONE_SIZE_HALF );
		USPRINTF("Zone Direction: 45~135");
	}
	//Case 3: >  135, < -135
	else if ( Zone_GetLastZoneDirection() < 2250 && Zone_GetLastZoneDirection() >= 1350 ) {
		retval = Zone_AddZone( start_x, start_y, des_x, des_y, uppestCell.X + gap - ZONE_SIZE_HALF,
		                                                       rightestCell.Y - gap + ZONE_SIZE_HALF );
		USPRINTF("Zone Direction: 135~225");
	}
	//Case 4: > -135, < -45
	else if ( Zone_GetLastZoneDirection() >= 2250 && Zone_GetLastZoneDirection() < 3150 ) {
		retval = Zone_AddZone( start_x, start_y, des_x, des_y, uppestCell.X + gap - ZONE_SIZE_HALF,
		                                                       leftestCell.Y + gap - ZONE_SIZE_HALF );
		USPRINTF("Zone Direction: 225~315");
	}
#endif

	if ( retval == -1 ) {
		USPRINTF("Cannot add zone! Go home!\n");
		CM_SetGoHome(0);
		return 1;
	}

	return retval;
}

/**
 * Add new zone
 * @param	start_x Start Cell x
 * @param	start_y Start Cell y
 * @param	exit_x End Cell x
 * @param	exit_y End Cell y
 * @param	zone_x Zone middle cell x
 * @param	zone_y Zone middle cell y
 * @return    1: successfully add; 0 or others: fail
 */
int8_t Zone_AddZone( int16_t start_x, int16_t start_y, int16_t exit_x, int16_t exit_y, int16_t zone_x, int16_t zone_y ) {
	uint8_t	retval = 0;

	int16_t i, j, k, clean = 0;
	CellState	cs;
	//Check
	if ( zones_cnt >= ZONE_MAX ) {
		USPRINTF("%s %d: failed to add zone, zone max cnt reached, (%d, %d).\n", __FUNCTION__, __LINE__, zone_x, zone_y);
		return -1;
	}

	//Check boundary
	USPRINTF("xRangeMin: %d\txRangeMax: %d\tyRangeMin: %d\tyRangeMax: %d\n",
	         xRangeMin, xRangeMax, yRangeMin, yRangeMax);
	if ( zone_x <= xRangeMin + BOUNDARY_INCREMENT + ZONE_SIZE_HALF ) {
		zone_x = xRangeMin + ZONE_SIZE_HALF + BOUNDARY_INCREMENT;
		USPRINTF("Out of xRangeMin! zone_x: %d\n", zone_x);
	}

	if ( zone_x >= xRangeMax - BOUNDARY_INCREMENT - ZONE_SIZE_HALF ) {
		zone_x = xRangeMax - ZONE_SIZE_HALF - BOUNDARY_INCREMENT;
		USPRINTF("Out of xRangeMax! zone_x: %d\n", zone_x);
	}

	if ( zone_y <= yRangeMin + BOUNDARY_INCREMENT + ZONE_SIZE_HALF ) {
		zone_y = yRangeMin + ZONE_SIZE_HALF + BOUNDARY_INCREMENT;
		USPRINTF("Out of yRangeMin! zone_y: %d\n", zone_y);
	}

	if ( zone_y >= yRangeMax - BOUNDARY_INCREMENT - ZONE_SIZE_HALF ) {
		zone_y = yRangeMax - ZONE_SIZE_HALF - BOUNDARY_INCREMENT;
		USPRINTF("Out of yRangeMax! zone_y: %d\n", zone_y);
	}

	//Zone
	zones[zones_cnt].zone.X = zone_x;
	zones[zones_cnt].zone.Y = zone_y;

	zones[zones_cnt].exitCell.X = exit_x;
	zones[zones_cnt].exitCell.Y = exit_y;

	zones[zones_cnt].entranceCell.X = start_x;
	zones[zones_cnt].entranceCell.Y = start_y;


	//Check if continual
	if ( isContinual == 1 && continualZonesSize < ZONE_MAX ) {
		continualZones[continualZonesSize] = zones_cnt;
		continualZonesSize++;
		uint8_t i;
		USPRINTF("Continual Zones: \n");
		for ( i = 0; i < continualZonesSize; ++i ) {
			USPRINTF("%d: Idx: %d\n", i, continualZones[i]);
		}
	}

	//Index++
	zones_cnt++;

	//Set zone boundary
	Zone_SetCurrentZoneBoundary(BLOCKED_BOUNDARY, 1);

	//Set wall follow status
	zoneWallFollowStatus = 2;

	//Clean obstcals in the current Zone
	if ( zones_cnt > ZONE_KEEP_OBSTACALS_SIZE + 1) {
		USPRINTF("%s %d Clean boundary! Current zone id: %d, Kept zone size: %d\n", __FUNCTION__, __LINE__,
		         zones_cnt - 1, ZONE_KEEP_OBSTACALS_SIZE);
		for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
			for (j = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; j <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; j++) {
				//If it is obstcal
				cs = Map_GetCell(MAP, zone_x + i, zone_y + j);
				if (cs >= BLOCKED && cs <= BLOCKED_CLIFF) {
					//Set the clean flag
					clean = 1;

					//Search not clean zones, check if this cell is in these zones
					for ( k = 0; k < ZONE_KEEP_OBSTACALS_SIZE; k++ ) {
						if ( ( (zones[zones_cnt - 2 - k].zone.X - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) <= (zone_x + i) ) &&
						     ( (zone_x + i) <= (zones[zones_cnt - 2 - k].zone.X + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) ) &&
						     ( (zones[zones_cnt - 2 - k].zone.Y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) <= (zone_y + j) ) &&
						     ( (zone_y + j) <= (zones[zones_cnt - 2 - k].zone.Y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) ) ) {
							clean = 0;
							//USPRINTF("(%d, %d)\n", zone_x + i, zone_y + j );
							break;
						}
					}

					//If it is not set to 0, it should set the obstcal to UNCLEANED
					if ( clean == 1 ) {
						Map_SetCell(MAP, cellToCount(zone_x + i), cellToCount(zone_y + j), CLEANED);
					}
				}
			}
		}
	}

	USPRINTF("%s %d: zone cnt: %d\n", __FUNCTION__, __LINE__, zones_cnt);
	retval = 1;
	return retval;
}


void Zone_SetZoneContinual( uint8_t val ) {
	isContinual = val;
}

uint8_t Zone_GetCurrentContinualZoneIdx(void) {
	if (continualZonesSize - 1 == 255 )
		return continualZones[0];
	else return continualZones[continualZonesSize - 1];
}


/**
 * Set current zone boundary
 * @param	state CellState, details in Map.h
 */
void Zone_SetCurrentZoneBoundary( CellState state, uint8_t flag ) {
	int16_t	i, j;

	zone_set_east_boundary(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y, state, flag);
	delay(50);
	zone_set_west_boundary(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y, state, flag);
	delay(50);
	zone_set_south_boundary(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y, state, flag);
	delay(50);
	zone_set_north_boundary(zones[zones_cnt - 1].zone.X, zones[zones_cnt - 1].zone.Y, state, flag);
	delay(50);

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; state == CLEANED && i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
		for (j = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; j <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; j++) {
			if (Map_GetCell(MAP, zones[zones_cnt - 1].zone.X + i, zones[zones_cnt - 1].zone.Y + j) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(zones[zones_cnt - 1].zone.X + i), cellToCount(zones[zones_cnt - 1].zone.Y + j), BLOCKED_OBS);
			}
		}
	}
}

/**
 * Set cell as zone entrance
 * @param cell cell
 * @param idx  index
 */
void Zone_SetZoneEntrance(Point16_t cell, uint8_t idx) {
	zones[idx].entranceCell = cell;
}

/**
 * Reset upest, donwest, rightest and leftest cell
 * @param cell cell
 * @param idx  index
 */
void Zone_ResetZoneUDRL(Point16_t cell) {
	uppestCell    =
	downestCell  =
	rightestCell =
	leftestCell  = cell;
}

/**
 * Get zone entrance cell
 * @param  Index
 * @return Entrance cell
 */
Point16_t Zone_GetZoneEntrance(uint8_t idx) {
	if ( idx < zones_cnt )
		return zones[idx].entranceCell;
	else return zones[0].entranceCell;
}

/**
 * Get current zone entrance cell
 * @return Entrance cell
 */
Point16_t Zone_GetCurrentZoneEntrance() {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return zones[0].entranceCell;
	else return zones[zones_cnt - 1].entranceCell;	
}


/**
 * Set cell as zone exit
 * @param cell cell
 * @param idx  index
 */
void Zone_SetZoneExit(Point16_t cell, uint8_t idx) {
	zones[idx].exitCell = cell;
}

/**
 * Get zone exit cell
 * @param  Index
 * @return Exit cell
 */
Point16_t Zone_GetZoneExit(uint8_t idx) {
	if ( idx < zones_cnt )
		return zones[idx].exitCell;
	else return zones[0].exitCell;
}

/**
 * Get current zone exit cell
 * @return Zone exit cell
 */
Point16_t Zone_GetCurrentZoneExit() {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return zones[0].exitCell;
	else return zones[zones_cnt - 1].exitCell;
}

/**
 * Get zone exit cell
 * @param  offset offset idx from the last zone
 * @return        Zone exit cell
 */
Point16_t Zone_GetZoneExitWithOffset(uint8_t offset) {
	if ( ((int16_t)zones_cnt - 1 - (int16_t)offset) < 0 )
		return zones[0].exitCell;
	else return zones[zones_cnt - 1 - offset].exitCell;
}

/**
 * Set the zone entrance heading with index
 * @param heading heading of robot
 * @param idx     Index
 */
void Zone_SetZoneEntranceRobotHeading(uint16_t heading, uint8_t idx) {
	zones[idx].entranceHeading = heading;
}

/**
 * Get zone entrance robot heading with index
 * @param  idx index
 * @return Robot heading
 */
uint16_t Zone_GetZoneEntranceRobotHeading(uint8_t idx) {
	if ( idx < zones_cnt )
		return zones[idx].entranceHeading;
	else return zones[0].entranceHeading;
}

/**
 * Get current zone entrance robot heading
 * @return Robot heading
 */
uint16_t Zone_GetCurrentZoneEntranceRobotHeading() {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return zones[0].entranceHeading;
	else return zones[zones_cnt - 1].entranceHeading;
}

/**
 * Set the zone exit heading with index
 * @param heading heading of robot
 * @param idx     Index
 */
void Zone_SetZoneExitRobotHeading(uint16_t heading, uint8_t idx) {
	zones[idx].exitHeading = heading;
}

/**
 * Get zone exit robot heading with index
 * @param  idx index
 * @return Robot heading
 */
uint16_t Zone_GetZoneExitRobotHeading(uint8_t idx) {
	if ( idx < zones_cnt )
		return zones[idx].exitHeading;
	else return zones[0].exitHeading;
}

/**
 * Get current zone exit robot heading
 * @return Robot heading
 */
uint16_t Zone_GetCurrentZoneExitRobotHeading(void) {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return zones[0].exitHeading;
	else return zones[zones_cnt - 1].exitHeading;
}

void zone_set_east_boundary(int16_t x, int16_t y, CellState state, uint8_t flag)
{
	int16_t i;
	CellState cell;

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
		if ( flag == 1 ) {
			if (state == BLOCKED_BOUNDARY) {
				cell = Map_GetCell(MAP, x + i, y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT);
				boundary_cell[0][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT] = (cell == BLOCKED_BOUNDARY ? UNCLEAN : cell);
				Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), state);
			} else if (state == CLEANED) {
				if (Map_GetCell(MAP, x + i, y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) == BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), boundary_cell[0][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT]);
				}
			} else {
				Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), state);
			}
		} else if ( state == BLOCKED_BOUNDARY && Map_GetCell(MAP, x + i, y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), state);
		}
	}
}

void zone_set_south_boundary(int16_t x, int16_t y, CellState state, uint8_t flag)
{
	int16_t i;
	CellState cell;

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
		if (flag == 1 ) {
			if (state == BLOCKED_BOUNDARY) {
				cell = Map_GetCell(MAP, x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT, y + i);
				boundary_cell[1][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT] = (cell == BLOCKED_BOUNDARY ? UNCLEAN : cell);
				Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), cellToCount(y + i), state);
			} else if (state == CLEANED) {
				if (Map_GetCell(MAP, x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT, y + i) == BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), cellToCount(y + i), boundary_cell[1][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT]);
				}
			} else {
				Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
		} else if ( state == BLOCKED_BOUNDARY && Map_GetCell(MAP, x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT, y + i) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), cellToCount(y + i), state);
		}

		if ( state != BLOCKED_BOUNDARY ) {
			if (Map_GetCell(MAP, x - ZONE_SIZE_HALF - 1 - BOUNDARY_INCREMENT, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF - 1 - BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
			if (Map_GetCell(MAP, x - ZONE_SIZE_HALF + 1 - BOUNDARY_INCREMENT, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - ZONE_SIZE_HALF + 1 - BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
		}
	}
}

void zone_set_west_boundary(int16_t x, int16_t y, CellState state, uint8_t flag)
{
	int16_t i;
	CellState cell;

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
		if ( flag == 1 ) {
			if (state == BLOCKED_BOUNDARY) {
				cell = Map_GetCell(MAP, x + i, y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT);
				boundary_cell[2][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT] = (cell == BLOCKED_BOUNDARY ? UNCLEAN : cell);
				Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), state);
			} else if (state == CLEANED) {
				if (Map_GetCell(MAP, x + i, y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) == BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), boundary_cell[2][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT]);
				}
			} else {
				Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), state);
			}
		} else if ( state == BLOCKED_BOUNDARY && Map_GetCell(MAP, x + i, y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - ZONE_SIZE_HALF - BOUNDARY_INCREMENT), state);
		}
	}
}

void zone_set_north_boundary(int16_t x, int16_t y, CellState state, uint8_t flag)
{
	int16_t i;
	CellState cell;

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT; i++) {
		if ( flag == 1 ) {
			if (state == BLOCKED_BOUNDARY) {
				cell = Map_GetCell(MAP, x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT, y + i);
				boundary_cell[3][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT] = (cell == BLOCKED_BOUNDARY ? UNCLEAN : cell);
				Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), cellToCount(y + i), state);
			} else if (state == CLEANED) {
				if (Map_GetCell(MAP, x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT, y + i) == BLOCKED_BOUNDARY) {
					Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), cellToCount(y + i), boundary_cell[3][i + ZONE_SIZE_HALF + BOUNDARY_INCREMENT]);
				}
			} else {
				Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
		} else if ( state == BLOCKED_BOUNDARY && Map_GetCell(MAP, x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT, y + i) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF + BOUNDARY_INCREMENT), cellToCount(y + i), state);
		}

		if ( state != BLOCKED_BOUNDARY ) {
			if (Map_GetCell(MAP, x + ZONE_SIZE_HALF - 1 + BOUNDARY_INCREMENT, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF - 1 + BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
			if (Map_GetCell(MAP, x + ZONE_SIZE_HALF + 1 + BOUNDARY_INCREMENT , y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x + ZONE_SIZE_HALF + 1 + BOUNDARY_INCREMENT), cellToCount(y + i), state);
			}
		}
	}
}
/**
 * Set Boundary
 * @param x      Cell x of zone
 * @param y      Cell y of zone
 * @param width  Width of zone
 * @param height Height of zone
 * @param state  Cell state
 * @param flag   1: force set; others, when state == BLOCKED_BOUNDARY, only can set the cell which is UNCLEAN
 */
void Zone_SetBoundary( int16_t x, int16_t y,
                                 uint16_t width, uint16_t height,
                                 CellState state, uint8_t flag ) {
	//East and West
	int16_t i;
	for (i = - width / 2 - 2; i <= width / 2 + 2; i++) {
		if ( flag == 1 ) {
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + height / 2 + 2), state);
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - height / 2 - 2), state);

		} else if ( state == BLOCKED_BOUNDARY &&
		            Map_GetCell(MAP, x + i, y + height / 2) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + height / 2 + 2), state);
			Map_SetCell(MAP, cellToCount(x + i), cellToCount(y - height / 2 - 2), state);
		}
	}

	//North and South
	for (i = - height / 2 - 2; i <= height / 2 + 2; i++) {
		if ( flag == 1 ) {
			Map_SetCell(MAP, cellToCount(x - width / 2 - 2), cellToCount(y + i), state);
			Map_SetCell(MAP, cellToCount(x + width / 2 + 2), cellToCount(y + i), state);
		} else if ( state == BLOCKED_BOUNDARY &&
		            Map_GetCell(MAP, x + width / 2, y + i) == UNCLEAN) {
			Map_SetCell(MAP, cellToCount(x - width / 2 - 2), cellToCount(y + i), state);
			Map_SetCell(MAP, cellToCount(x + width / 2 + 2), cellToCount(y + i), state);
		}

		if ( state != BLOCKED_BOUNDARY ) {
			if (Map_GetCell(MAP, x + width / 2 - 1 + 2, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - width / 2 - 1), cellToCount(y + i), state);
			}
			if (Map_GetCell(MAP, x + width / 2 + 1 + 2, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - width / 2 + 1), cellToCount(y + i), state);
			}
			if (Map_GetCell(MAP, x - width / 2 - 1 - 2, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - width / 2 - 1), cellToCount(y + i), state);
			}
			if (Map_GetCell(MAP, x - width / 2 + 1 - 2, y + i) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(x - width / 2 + 1), cellToCount(y + i), state);
			}
		}
	}
}

/**
 * Return status of Zone wall follow
 * @return  0: All map cleaning;
 *          1: Finish cleaning / Middle zone cleaning;
 *          2: Normal zone cleaning
 *          3: Trapped mode, find path to trapped zone
 */
uint8_t Zone_GetZoneWallFollowStatus() {
	return zoneWallFollowStatus;
}

/**
 * Set zone wall follow type
 * @param val type 0: All map cleaning;
 *                 1: Finish cleaning / Middle zone cleaning;
 *                 2: Normal zone cleaning
 *                 3: Trapped mode, find path to trapped zone
 */
void Zone_SetZoneWallFollowStatus(uint8_t val) {
	zoneWallFollowStatus = val;
}

/**
 * Get zone with index
 * @param  idx index
 * @return     zone
 */
Point16_t Zone_GetZone(uint8_t idx) {
	if ( idx < zones_cnt )
		return zones[idx].zone;
	return zones[0].zone;
}

/**
 * Get current zone
 * @return     zone
 */
Point16_t Zone_GetCurrentZone() {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return zones[0].zone;
	else return zones[zones_cnt - 1].zone;
}

/**
 * Get zone
 * @param  offset offset idx from last zone
 * @return        zone
 */
Point16_t Zone_GetZoneWithOffset(uint8_t offset) {
	if ( ((int16_t)zones_cnt - 1 - (int16_t)offset) < 0 )
		return zones[0].zone;
	else return zones[zones_cnt - 1 - offset].zone;
}

uint8_t Zone_GetZoneSize() {
	return zones_cnt;
}

uint8_t Zone_GetCurrentZoneIdx() {
	if ( (uint8_t)(zones_cnt - 1) == (uint8_t)255 )
		return 0;
	else return zones_cnt - 1;
}

void Zone_RemoveCurrentZone()
{
	if (zones_cnt != 0) {
		zones_cnt--;
	}

	//Check if continual
	if ( continualZonesSize != 0 ) {
		continualZonesSize--;
	}
}

uint8_t Zone_IsOverZoneCount() {
	if ( zones_cnt >= ZONE_MAX )
		return 1;
	else return 0;
}


uint16_t Zone_GetLastZoneDirection() {
	return course2dest( zones[zones_cnt - 1].entranceCell.X,
	                   	zones[zones_cnt - 1].entranceCell.Y,
	                   	zones[zones_cnt - 1].exitCell.X,
	                   	zones[zones_cnt - 1].exitCell.Y );
}

uint8_t Zone_GetTrappedZoneSize() {
	return trappedZoneSize;
}

uint8_t Zone_GetTrappedZoneIdx( uint8_t id ) {
	return trappedZoneIdx[id];
}

void Zone_DeleteTrappedZoneIdx( uint8_t id ) {
	//TODO:
	uint8_t i;
	for ( i = id; i < trappedZoneSize - 1; i++ ) {
		trappedZoneIdx[i] = trappedZoneIdx[i + 1];
	}
	trappedZoneSize--;

	USPRINTF("%s %d Trapped zones: size: %d\n", __FUNCTION__, __LINE__, trappedZoneSize);
	for ( i = 0; i < trappedZoneSize; ++i ) {
		USPRINTF("Idx %d: %d\n", i, trappedZoneIdx[i]);
	}
}

void Zone_AddTrappedZoneIdx() {
	uint8_t i;

	trappedZoneIdx[trappedZoneSize] = zones_cnt - 1;
	trappedZoneSize++;

	USPRINTF("%s %d Trapped zones: size: %d\n", __FUNCTION__, __LINE__, trappedZoneSize);
	for ( i = 0; i < trappedZoneSize; ++i ) {
		USPRINTF("Idx %d: %d\n", i, trappedZoneIdx[i]);
	}
}

void Zone_GetRange(int16_t *x_min, int16_t *x_max, int16_t *y_min, int16_t *y_max)
{
	*x_min = downestCell.X;
	*x_max = uppestCell.X;
	*y_min = rightestCell.Y;
	*y_max = leftestCell.Y;
}

void Zone_ResetPathBlocks(void) {
	int16_t	i, j;

	for (i = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT + 1; i <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT - 1; i++) {
		for (j = -ZONE_SIZE_HALF - BOUNDARY_INCREMENT + 1; j <= ZONE_SIZE_HALF + BOUNDARY_INCREMENT - 1; j++) {
			if (Map_GetCell(MAP, zones[zones_cnt - 1].zone.X + i, zones[zones_cnt - 1].zone.Y + j) == BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, cellToCount(zones[zones_cnt - 1].zone.X + i), cellToCount(zones[zones_cnt - 1].zone.Y + j), BLOCKED_OBS);
			}
		}
	}
}
#endif
