
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
///////////////////////////////

int num_of_T;
int count;
struct lock lock2;
struct condition condition2;
//////////////////////////////////////////
/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
		/* to B */
		{{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
		/* to C */
		{{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
		/* to D */
		{{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);  //&&&&&&&&&&&&&&&&maybe not termination, but when out of critical section .$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
			return 0;
		}
	}

	/* lock next position */
	//lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
	
	
		if (start == 'A' - 'A') // from A
		{
			if (dest == 'B' - 'A') // to B
			{
				if (pos_cur.row == 4 && pos_cur.col == 1) 
				{
					if(lock_try_acquire(&vi->map_locks[4][2])==true)	
					{
						
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
						vi->position = pos_next;
					}
					else return -1;
				}
				else if (pos_cur.row == 4 && pos_cur.col == 2) 
				{
					
					lock_acquire(&vi->map_locks[5][2]);
					lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					vi->position = pos_next;
				}
				else
				{	
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING; 
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'C' - 'A') // to C
			{
				if (pos_cur.row == 4 && pos_cur.col == 1) // lock
				{
					if(lock_try_acquire(&vi->map_locks[4][2])==true)
					{
						if(lock_try_acquire(&vi->map_locks[4][3])==true)
						{
							if(lock_try_acquire(&vi->map_locks[4][4])==true)
							{
								lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
								vi->position = pos_next;
							}
							else
							{
								lock_release(&vi->map_locks[4][2]);
								lock_release(&vi->map_locks[4][3]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[4][2]);
							return -1;
						}	
					}
					else return -1;			
				}
				else if (pos_cur.row == 4 && pos_cur.col == 2)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 4 && pos_cur.col == 3)
				{
						vi->position = pos_next;	
				}
				else if(pos_cur.row == 4 && pos_cur.col == 4)
				{
						lock_acquire(&vi->map_locks[4][5]);
						lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]); lock_release(&vi->map_locks[4][4]);
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'D' - 'A') // to D
			{
				if (pos_cur.row == 4 && pos_cur.col == 1) // lock
				{
					if(lock_try_acquire(&vi->map_locks[4][2])==true)
					{
						if(lock_try_acquire(&vi->map_locks[4][3])==true)
						{
							if(lock_try_acquire(&vi->map_locks[4][4])==true)
							{
								if(lock_try_acquire(&vi->map_locks[3][4])==true)
								{
									if(lock_try_acquire(&vi->map_locks[2][4])==true)
									{
										lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
										vi->position = pos_next;
									}
									else
									{
										lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]); lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]);
										return -1;
									}
								}
								else
								{
									lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]); lock_release(&vi->map_locks[4][4]);
									return -1;
								}
							}
							else
							{
								lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[4][2]);
							return -1;
						}
						
					}
					else return -1;		
				}
				else if (pos_cur.row == 4 && pos_cur.col == 2)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 4 && pos_cur.col == 3)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 4 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 3 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 4)
				{
						lock_acquire(&vi->map_locks[1][4]);
						lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]); lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]); lock_release(&vi->map_locks[2][4]);
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
						
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
		}
		else if (start == 'B' - 'A') // from B
		{
			if (dest == 'A' - 'A') // to A
			{
				if (pos_cur.row == 5 && pos_cur.col == 4) // lock
				{
					if(lock_try_acquire(&vi->map_locks[4][4])==true)
					{
						if(lock_try_acquire(&vi->map_locks[3][4])==true)
						{
							if(lock_try_acquire(&vi->map_locks[2][4])==true)
							{
								if(lock_try_acquire(&vi->map_locks[2][3])==true)
								{
									if(lock_try_acquire(&vi->map_locks[2][2])==true)
									{
										lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
										vi->position = pos_next;
									}
									else
									{
										lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]); lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]);
										return -1;
									}
								}
								else
								{
									lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]); lock_release(&vi->map_locks[2][4]);
									return -1;
								}
							}
							else
							{
								lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[4][4]);
							return -1;
						}
					}
					 else return -1;					
				}
				else if (pos_cur.row == 4 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 3 && pos_cur.col == 4)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 2 && pos_cur.col == 4)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 2 && pos_cur.col == 3)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 2 && pos_cur.col == 2)
				{
						lock_acquire(&vi->map_locks[2][1]);
						lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]); lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]); lock_release(&vi->map_locks[2][2]);	
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'C' - 'A') // to C
			{
				if (pos_cur.row == 5 && pos_cur.col == 4) 
				{
					if(lock_try_acquire(&vi->map_locks[4][4])==true)
					{
						
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
						vi->position = pos_next;
					}
					else return -1;
					
				}
				else if (pos_cur.row == 4 && pos_cur.col == 4) 
				{
					
					lock_acquire(&vi->map_locks[4][5]);
					lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'D' - 'A') // to D
			{
				if (pos_cur.row == 5 && pos_cur.col == 4) // lock
				{
					if(lock_try_acquire(&vi->map_locks[4][4])==true)
					{
						if(lock_try_acquire(&vi->map_locks[3][4])==true)
						{
							if(lock_try_acquire(&vi->map_locks[2][4])==true)
							{
								lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
								vi->position = pos_next;
							}
							else
							{
								lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[4][4]);
							return -1;
						}
						
					}
					else return -1;
				}
				else if (pos_cur.row == 4 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 3 && pos_cur.col == 4)
				{
						vi->position = pos_next;	
				}
				else if(pos_cur.row == 2 && pos_cur.col == 4)
				{
						lock_acquire(&vi->map_locks[1][4]);
						lock_release(&vi->map_locks[4][4]); lock_release(&vi->map_locks[3][4]); lock_release(&vi->map_locks[2][4]);
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
		}
		else if (start == 'C' - 'A') // from C
		{
			if (dest == 'A' - 'A') // to A
			{
				if (pos_cur.row == 2 && pos_cur.col == 5) // lock
				{
					if(lock_try_acquire(&vi->map_locks[2][4])==true)
					{
						if(lock_try_acquire(&vi->map_locks[2][3])==true)
						{
							if(lock_try_acquire(&vi->map_locks[2][2])==true)
							{
								lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
								vi->position = pos_next;
							}
							else
							{
								lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[2][4]);
							return -1;
						}
					}
					else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 3)
				{
						vi->position = pos_next;	
				}
				else if(pos_cur.row == 2 && pos_cur.col == 2)
				{
						lock_acquire(&vi->map_locks[2][1]);
						lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]); lock_release(&vi->map_locks[2][2]);
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'B' - 'A') // to B
			{
				if (pos_cur.row == 2 && pos_cur.col == 5) // lock
				{
					if(lock_try_acquire(&vi->map_locks[2][4])==true)
					{
						if(lock_try_acquire(&vi->map_locks[2][3])==true)
						{
							if(lock_try_acquire(&vi->map_locks[2][2])==true)
							{
								if(lock_try_acquire(&vi->map_locks[3][2])==true)
								{
									if(lock_try_acquire(&vi->map_locks[4][2])==true)
									{
										lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
										vi->position = pos_next;
									}
									else
									{
										lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]); lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]);
										return -1;
									}
								}
								else
								{
									lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]); lock_release(&vi->map_locks[2][2]);
									return -1;
								}
							}
							else
							{
								lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[2][4]);
							return -1;
						}
					}
					else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 4)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 3)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 2 && pos_cur.col == 2)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 3 && pos_cur.col == 2)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 4 && pos_cur.col == 2)
				{
						lock_acquire(&vi->map_locks[5][2]);
						lock_release(&vi->map_locks[2][4]); lock_release(&vi->map_locks[2][3]); lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]); lock_release(&vi->map_locks[4][2]);	
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'D' - 'A') // to D
			{
				if (pos_cur.row == 2 && pos_cur.col == 5) 
				{
					if(lock_try_acquire(&vi->map_locks[2][4])==true)
					{
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
						vi->position = pos_next;
					}
					else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 4) 
				{
					
					lock_acquire(&vi->map_locks[1][4]);
					lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
		}
		else if (start == 'D' - 'A') // from D
		{
			if (dest == 'A' - 'A') // to A
			{
				if (pos_cur.row == 1 && pos_cur.col == 2) 
				{
					if(lock_try_acquire(&vi->map_locks[2][2])==true)
					{
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
						vi->position = pos_next;
					}
					else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 2) 
				{
					
					lock_acquire(&vi->map_locks[2][1]);
					lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'B' - 'A') // to B
			{
				if (pos_cur.row == 1 && pos_cur.col == 2) // lock
				{
					if(lock_try_acquire(&vi->map_locks[2][2])==true)
					{
						if(lock_try_acquire(&vi->map_locks[3][2])==true)
						{
							if(lock_try_acquire(&vi->map_locks[4][2])==true)
							{
								lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
								vi->position = pos_next;
							}
							else
							{
								lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[2][2]);
							return -1;
						}
					}
					 else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 2)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 3 && pos_cur.col == 2)
				{
						vi->position = pos_next;	
				}
				else if(pos_cur.row == 4 && pos_cur.col == 2)
				{
						lock_acquire(&vi->map_locks[5][2]);
						lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]); lock_release(&vi->map_locks[4][2]);
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
			else if (dest == 'C' - 'A') // to C
			{
				if (pos_cur.row == 1 && pos_cur.col == 2) // lock
				{
					if(lock_try_acquire(&vi->map_locks[2][2])==true)
					{
						if(lock_try_acquire(&vi->map_locks[3][2])==true)
						{
							if(lock_try_acquire(&vi->map_locks[4][2])==true)
							{
								if(lock_try_acquire(&vi->map_locks[4][3])==true)
								{
									if(lock_try_acquire(&vi->map_locks[4][4])==true)
									{
										lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
										vi->position = pos_next;
									}
									else
									{
										lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]); lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]);
										return -1;
									}
								}
								else
								{
									lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]); lock_release(&vi->map_locks[4][2]);
									return -1;
								}
							}
							else
							{
								lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]);
								return -1;
							}
						}
						else
						{
							lock_release(&vi->map_locks[2][2]);
							return -1;
						}
					}
					 else return -1;
				}
				else if (pos_cur.row == 2 && pos_cur.col == 2)
				{
						vi->position = pos_next;
				}
				else if (pos_cur.row == 3 && pos_cur.col == 2)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 4 && pos_cur.col == 2)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 4 && pos_cur.col == 3)
				{
						vi->position = pos_next;	
				}
				else if (pos_cur.row == 4 && pos_cur.col == 4)
				{
						lock_acquire(&vi->map_locks[4][5]);
						lock_release(&vi->map_locks[2][2]); lock_release(&vi->map_locks[3][2]); lock_release(&vi->map_locks[4][2]); lock_release(&vi->map_locks[4][3]); lock_release(&vi->map_locks[4][4]);	
						vi->position = pos_next;
				}
				else
				{
					if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])==false)
					{
						return -1;
					}
					
					if (vi->state == VEHICLE_STATUS_READY)
					{
						/* start this vehicle */
						vi->state = VEHICLE_STATUS_RUNNING;
					}
					else
					{
						/* release current position */
						lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
					}
					vi->position = pos_next;
				}
			}
		}

	
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	count=thread_cnt;
	num_of_T=thread_cnt;
	lock_init(&lock2);
	cond_init(&condition2);
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++; 
		}

		

		/* unitstep change! */
				lock_acquire(&lock2);

				count--;
				/* termination condition. */
				if(res==0&&count!=0)
				{
					num_of_T--;
					lock_release(&lock2);
					break;
				}
				else if (res == 0&&count==0)
				{
					num_of_T--;
					crossroads_step++;
					unitstep_changed();
					count = num_of_T;
					cond_broadcast(&condition2,&lock2);
					lock_release(&lock2);
					break;
				}

				else if(count==0)
				{

					crossroads_step++;
					unitstep_changed();
					count = num_of_T;
					cond_broadcast(&condition2,&lock2);
					lock_release(&lock2);
				}
				else
				{
					cond_wait(&condition2,&lock2);
					if(lock_held_by_current_thread (&lock2))
					{lock_release(&lock2);}
				}

					
			
	
	}
	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
