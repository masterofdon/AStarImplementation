//A* Pathfinding Algorithm Implementation
//Coded by: Ahmet Erdem Ekin
//Start Date:17/2/2013
//End Date:20/2/2013
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct node
{
  struct point* pnt;
  struct node* parentNode;
  float g;
  float h;       
};
struct point
{
  int x;
  int y;     
};
//------------------------------------------------------------------------------
//the function that operates during path finding process.
struct node* AStarAlgorithm(struct node* current,struct node* goal,struct node **array,int l1,struct node **closed,
int l2);
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//the function to find the neighboring nodes.to be called inside AStarAlgorithm
//returns the number of elements inside the array...
//parameters:CURRENT NODE,LIST OF NODES TO BE INSPECTED,LENGTH OF THE LIST
int ExpandNode(struct node* current,struct node **array,int l1,struct node **closed,int l2);
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//finds the least total costed node and returns a pointer for that node...
//paramenters:LIST OF NODES TO BE INSPECTED,LENGTH OF THE LIST
int FindTheLeastCosted(struct node **array,int l1);
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//calcualtes the total cost of each node in the array...
//parameters: GOAL NODE,LIST OF NODES TO BE INSPECTED,LENGTH OF THE LIST
void CalculateTheTotalCost(struct node* goalNode,struct node **array,int l1);
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//reconstructs the path from the beginning to the goal.prints out the path...
void ReconstructThePath(struct node* goalNode);
//------------------------------------------------------------------------------
int main()
{
    //create START_NODE=>sP and set the data...
    struct point* sP = (struct point*)malloc(sizeof(struct point));
    sP->x = 0,sP->y = 0;
    struct node* startNode = (struct node*)malloc(sizeof(struct node));
    startNode->pnt = sP,startNode->parentNode = NULL,startNode->g=0;startNode->h=0;

    //create GOAL_NODE=>gP and set the data...
    struct point* gP = (struct point*)malloc(sizeof(struct point));
    gP->x = 51,gP->y = 11;
    struct node* goalNode = (struct node*)malloc(sizeof(struct node));
    goalNode->pnt = gP,goalNode->parentNode = NULL,goalNode->g=0,startNode->h=0;

    //create OPEN_LIST=>array as a pointer to a pointer that is pointing to an array...
    struct node **array = (struct node**)malloc(sizeof(struct node*));
    *array = NULL;
    
    //create CLOSE_LIST=>closedArray as a pointer to a pointer that is pointing to an array...
    struct node **closedArray = (struct node**)malloc(sizeof(struct node*));
    *closedArray = NULL;
    (*closedArray) = (struct node*)realloc((*closedArray),sizeof(struct node));
    memcpy(&((*closedArray)[0]),startNode,sizeof(struct node));
    
    //create LAST_OBTAINED_NODE=>finished and set it to the result returned from the recursive A_STAR_ALGORITHM_FUNC
    struct node* finished = AStarAlgorithm(startNode,goalNode,array,0,closedArray,1);

    //call RECONSTRUCT_FUNC=>ReconstructThePath with finished as parameter...
    ReconstructThePath(finished);
    
    //wait for user input...
    getch();

    //QUIT
    return 0;   
}
struct node* AStarAlgorithm(struct node* current,struct node* goal,struct node **array,int l1,struct node **closed,int l2)
{
  int i,j,nextIndex;

  //call EXPAND_NODE_FUNC ExpandNode and set LEN_OF_OPEN_LIST to the value returned from that function...
  l1 = ExpandNode(current,array,l1,closed,l2);

  //call CALCULATE_COST=>CalculateTheTotalCost function to set the G_FUNC=>g cost of each element in OPEN_LIST=>array...  
  CalculateTheTotalCost(goal,array,l1);

  //call FIND_LEAST_NODE=>FindTheLeastCosted function to obtain the index of least costed node in the OPEN_LIST=>array...
  nextIndex = FindTheLeastCosted(array,l1);

  struct node *nextNode = &((*array)[nextIndex]);

  //create a new temporary list which is intended to be the next OPEN_LIST=>array
  //without the NEXT_NODE=>nextNode inside.
  struct node *tempArray = (struct node*)calloc(l1-1,sizeof(struct node));
  for(i = 0,j=0;i<l1;i++)
    if(i != nextIndex) memcpy(&tempArray[j],&((*array)[i]),sizeof(struct node)),j++;
  l1--,l2++;
  *array = tempArray;

  //declare NEXT_NODE=>nexNode to be visited by adding the node to the CLOSE_LIST=>closed...
  *closed = (struct node*)realloc((*closed),l2*(sizeof(struct node)));
  memcpy(&((*closed)[l2-1]),nextNode,sizeof(struct node));

  //check whether the NEXT_NODE=>nextNode is the GOAL_NODE=>goal by comparing its coordinates... 
  //if so => return NEXT_NODE=>nextNode
  //else call A_STAR_ALGORITHM_FUNC0>AStarAlgorithm with CURRENT_NODE=>nextNode,
  //GOAL_NODE=>goal, OPEN_LIST=>array, LEN_OPEN_LIST=>l1,CLOSE_LIST=>closed,LeN_CLOSE_LIST=>l2... 
  if(nextNode->pnt->x == goal->pnt->x && nextNode->pnt->y == goal->pnt->y)  return nextNode;
  return AStarAlgorithm(nextNode,goal,array,l1,closed,l2);
}
int ExpandNode(struct node* current,struct node **array,int l1,struct node **closed,int l2)
{   
   int i,j,count,found;
   count = 0;
   //--FIRST-PHASE--:EXPAND ALL THE NEIGHBORING NODES...
   struct node* tempArray;
   tempArray = (struct node*)calloc(8,sizeof(struct node));   
   for(i = 0;i<8;i++)
   {
      tempArray[i].pnt = (struct point*)malloc(sizeof(struct point));
      if(i == 0)  tempArray[i].pnt->x = current->pnt->x-1,tempArray[i].pnt->y = current->pnt->y+1,tempArray[i].g = current->g+1.4;
      else if(i == 1)   tempArray[i].pnt->x = current->pnt->x-1,tempArray[i].pnt->y = current->pnt->y,tempArray[i].g = current->g+1.0f;
      else if(i == 2)   tempArray[i].pnt->x = current->pnt->x-1,tempArray[i].pnt->y = current->pnt->y-1,tempArray[i].g = current->g+1.4f;
      else if(i == 3)   tempArray[i].pnt->x = current->pnt->x,tempArray[i].pnt->y = current->pnt->y+1,tempArray[i].g = current->g+1.0f;
      else if(i == 4)   tempArray[i].pnt->x = current->pnt->x,tempArray[i].pnt->y = current->pnt->y-1,tempArray[i].g = current->g+1.0f;
      else if(i == 5)   tempArray[i].pnt->x = current->pnt->x+1,tempArray[i].pnt->y = current->pnt->y+1,tempArray[i].g = current->g+1.4f;
      else if(i == 6)   tempArray[i].pnt->x = current->pnt->x+1,tempArray[i].pnt->y = current->pnt->y,tempArray[i].g = current->g+1.0f;
      else if(i == 7)   tempArray[i].pnt->x = current->pnt->x+1,tempArray[i].pnt->y = current->pnt->y-1,tempArray[i].g = current->g+1.4f;
      tempArray[i].parentNode = current;
   }

   //--SECOND-PHASE--:check whether current node's children are inside the array.
   //if so, do not include those nodes to the expanded array. 
   for(j = 0;j<8;j++)
   {
      found = 0;           
      for(i = 0;i<l1;i++) if((*array)[i].pnt->x == tempArray[j].pnt->x && (*array)[i].pnt->y == tempArray[j].pnt->y) found++;
      for(i = 0;i<l2;i++) if(tempArray[j].pnt->x == (*closed)[i].pnt->x && (*closed)[i].pnt->y == tempArray[j].pnt->y) found++; 
      //--THIRD-PHASE--:expand the array with the additional neighbors.
      if(found == 0)
      {
        count++;
        int total = (l1+count);
        *array = (struct node*)realloc((*array),total*(sizeof(struct node)));
        memcpy(&((*array)[total-1]),&tempArray[j],sizeof(struct node));              
      }      
   } 
   return count+l1;     
}
void CalculateTheTotalCost(struct node* goalNode,struct node **array,int l1)
{
    int i,difx,dify;
    //h(x,y) calculation...h(x,y) = ((x2-x1)^2 + (y2-y1)^2)^(1/2)
    for(i = 0;i<l1;i++)
    {
        difx = (*array)[i].pnt->x - goalNode->pnt->x;
        dify = (*array)[i].pnt->y - goalNode->pnt->y;
        (*array)[i].h = ((float)sqrt(pow(difx,2) + pow(dify,2)));
    }
}
int FindTheLeastCosted(struct node **array,int l1)
{
    int i,min,minIndex;
    min = (*array)[0].g + (*array)[0].h;
    minIndex = 0;
    for(i= 1;i<l1;i++)
    {
        if((*array)[i].g + (*array)[i].h < min) min = (*array)[i].g + (*array)[i].h,minIndex = i;
    }
    return minIndex;
}
void ReconstructThePath(struct node* goalNode)
{
     struct node* current = goalNode;
     struct point* ptr = NULL;
     int steps = 0,i;
     while(current->parentNode != NULL)
     {
          steps++;
          ptr = (struct point*)realloc(ptr,steps*sizeof(struct point));
          memcpy(&ptr[steps-1],current->pnt,sizeof(struct point));
          current = current->parentNode;                    
     }
     for(i=steps;i>=1;i--)
     {
        printf("(%d,%d)",ptr[i-1].x,ptr[i-1].y);
        if(i>1)  printf("=>");  
     }
}
