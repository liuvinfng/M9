#include "PathPlanning.h"
#include "PathPlanning-Map.h"
#include "Map-Zip.h"

uint8_t map_zipped[MAP_SIZE][(MAP_SIZE + 7) / 8];

int16_t xMin, xMax, yMin, yMax;
int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

void map_zip_set_cell(int16_t x, int16_t y, uint8_t val);
uint8_t map_zip_get_cell(int16_t x, int16_t y);

void map_zip(void)
{
	int16_t i, j;

	xMin = xMax = yMin = yMax = 0;
	xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
	xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
	yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
	yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));

	for (i = 0; i < MAP_SIZE; ++i) {
		for (j = 0; j < (MAP_SIZE + 1) / 2; ++j) {
			map_zip_set_cell(i, j, Map_GetCell(MAP, i, j) > 1 ? 1 : 0);
		}
	}
	USPRINTF("%s %d: %d %d %d %d\r\n", __FUNCTION__, __LINE__, xMin, xMax, yMin, yMax);
	USPRINTF("%s %d: %d %d %d %d\r\n", __FUNCTION__, __LINE__, xRangeMin, xRangeMax, yRangeMin, yRangeMax);
}

void map_unzip(void)
{
	int16_t i, j;

	USPRINTF("%s %d: %d %d %d %d\r\n", __FUNCTION__, __LINE__, xMin, xMax, yMin, yMax);
	USPRINTF("%s %d: %d %d %d %d\r\n", __FUNCTION__, __LINE__, xRangeMin, xRangeMax, yRangeMin, yRangeMax);
	for (i = xRangeMin; i < xRangeMax; ++i) {
		for (j = yRangeMin; j < yRangeMax; ++j) {
			Map_SetCell(MAP, cellToCount(i), cellToCount(j), map_zip_get_cell(i, j) == 0 ? 0 : 3);
		}
	}
}

void map_zip_set_cell(int16_t x, int16_t y, uint8_t val)
{
	int16_t ROW, COLUMN;

	if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
		USPRINTF("%s %d: %d %d %d %d %d %d\r\n", __FUNCTION__, __LINE__, xRangeMin, xRangeMax, yRangeMin, yRangeMax);
		if(x < xMin) {
			xMin = x;
			xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
			xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
		} else if(x > xMax) {
			xMax = x;
			xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
			xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
		}
		if(y < yMin) {
			yMin = y;
			yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
			yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));
		} else if(y > yMax) {
			yMax = y;
			yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
			yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));
		}

		ROW = x + MAP_SIZE + MAP_SIZE / 2;
		ROW %= MAP_SIZE;
		COLUMN = y + MAP_SIZE + MAP_SIZE / 2;
		COLUMN %= MAP_SIZE;

		map_zipped[x][y / 8] = (map_zipped[x][y / 8] & (~(0x1 << (7 - (y % 8))))) | (val << (7 - (y % 8)));
	}
}

uint8_t map_zip_get_cell(int16_t x, int16_t y)
{
	uint8_t	val = 1;

	USPRINTF("%s %d: %d %d %d %d %d %d\r\n", __FUNCTION__, __LINE__, x, xRangeMin, xRangeMax, y, yRangeMin, yRangeMax);
	if (x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
		x += MAP_SIZE + MAP_SIZE / 2;
		x %= MAP_SIZE;
		y += MAP_SIZE + MAP_SIZE / 2;
		y %= MAP_SIZE;

		val = ((map_zipped[x][y / 8] & (0x1 << (7 - (y % 8)))) != 0x0 ? 1 : 0);
	}

	return val;
}
