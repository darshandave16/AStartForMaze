
//Solving Maze with A* with different heuristics

let scaleFactor = 20;
let bAllowDiagonal = false;
let grid = [];
let maxX = 0;
let maxY = 0;

// #Region Heuristic fuctions implementation
function coumputeEuclideanDistance(x1,y1,x2,y2)
{
    return Math.sqrt(Math.pow((x2-x1),2) + Math.pow((y2 - y1),2));
}
function squaredEuclideanDistance(x1,y1,x2,y2)
{
    return Math.pow((x2-x1),2) + Math.pow((y2 - y1),2);
}
function computechebyshevDistance(x1,y1,x2,y2)
{
    return Math.max(Math.abs(x2-x1), Math.abs(y2 - y1));
}
function computeManhattanDistance(x1,y1,x2,y2)
{
    return Math.abs(x2-x1) + Math.abs(y2 - y1);
}
//# end Region

// # Region
// Using A* with diffrent heuristics for drawing Output
// and measuring time to compute using diffrent heuristics
function aStartByH0()
{
    let path1 =[];
    let startime = (new Date()).getTime();
    path1 = aSTAR(grid[0][0],1);
    console.log("Time taken for h = 0 : " + ((new Date()).getTime() - startime) + " ms and " 
    + "number of states visited " + path1.length);

    for(let p =0; p < path1.length -1; p++)
    {
        path1[p].ele.style.backgroundColor = "rgba(245, 140, 115, 0.8)";
    }
}

function aStartByManhattan()
{
    let path2 =[];
    startime = (new Date()).getTime();
    path2 = aSTAR(grid[0][0],2);
    console.log("Time taken for ManhattanDistance: " + ((new Date()).getTime() - startime) + " ms and " 
    + "number of states visited " + path2.length);

    for(let p =0; p < path2.length -1; p++)
    {
        path2[p].ele.style.backgroundColor = "rgba(125, 140, 185, 0.8)";
    }
}
function aStartByEuclidean()
{
    let path3 =[];

    startime = (new Date()).getTime();
    path3 = aSTAR(grid[0][0],3);
    console.log("Time taken for EuclideanDistance: " + ((new Date()).getTime() - startime) + " ms" 
    + "number of states visited " + path3.length);

    for(let p =0; p < path3.length -1; p++)
    {
        path3[p].ele.style.backgroundColor = "rgba(124, 240, 175, 0.8)";
    }
}
function aStartBySquaredEuclidean()
{
    let path4 =[];

    startime = (new Date()).getTime();
    path4 = aSTAR(grid[0][0],4);
    console.log("Time taken for squaredEuclideanDistance : " + ((new Date()).getTime() - startime) + " ms" 
    + "number of states visited " + path4.length);

    for(let p =0; p < path4.length -1; p++)
    {
        path4[p].ele.style.backgroundColor = "rgba(145, 189, 165, 0.8)";
    }
}
function aStartBychebyshev()
{
    let path5 =[];

    startime = (new Date()).getTime();
    path5 = aSTAR(grid[0][0],5);
    console.log("Time taken for ChebyshevDistance : " + ((new Date()).getTime() - startime) + " ms" 
    + "number of states visited " + path5.length);

    for(let p =0; p < path5.length -1; p++)
    {
        path5[p].ele.style.backgroundColor = "rgba(100, 140, 210, 0.8)";
    }
}
function aStartByFreedomOfMovement()
{
    let path6 =[];

    startime = (new Date()).getTime();
    path6 = aSTAR(grid[0][0],6);
    console.log("Time taken for Degree of freedom for movement : " + ((new Date()).getTime() - startime) + " ms"
    + "number of states visited " + path6.length);

    for(let p =0; p < path6.length -1; p++)
    {
        path6[p].ele.style.backgroundColor = "rgba(175, 134, 145, 0.8)";
    }
}

//# end Region

//Implemntaion of A *
//** Param startState -> start of the maze */
//** Param ntype -> type of heuristic used */
function aSTAR(startState, ntype) 
{
    lstFrontier = [];
    lstReached = [];
    
    lstFrontier.push(startState);
    startState.f = 0;
    startState.g = 0;
    
    // chose a node from frontier with minimum f(x) value
    while (lstFrontier.length > 0) 
    {
      var lowInd = 0;
      for(var i= 0; i<lstFrontier.length; i++) {
        if(lstFrontier[i].f < lstFrontier[lowInd].f) { lowInd = i; }
      }
      var currentNode = lstFrontier[lowInd];

        for(let j = 0; j < lstFrontier.length; j++)
        {
            if(lstFrontier[j] == currentNode)
            {
                lstFrontier.splice(j,1);
            }
        }
        lstReached.push(currentNode);

        // check if goal is reached
        if(currentNode == grid[maxY -1][maxX -1]) {
            var curr = currentNode;
            var ret = [];
            while(curr.parent) {
            ret.push(curr);
            curr = curr.parent;
            }
            return ret.reverse();
        }

        var neighbors = currentNode.lstNeighbours;

        for(var i= 0; i<neighbors.length;i++) {
            var neighbor = neighbors[i];
            if(lstReached.indexOf(neighbor) != -1 || neighbor.bBlock) {
                continue;
            }

            var gScore = currentNode.g + 1;
            var gScoreIsBest = false;

            if(lstFrontier.indexOf(neighbor) == -1) {

                gScoreIsBest = true;
                if(ntype == 1)
                    neighbor.h = 0;
                if(ntype == 2)
                    neighbor.h = neighbor.manhattanDistance
                if(ntype == 3)
                    neighbor.h = neighbor.euclideanDistance
                if(ntype == 4)
                    neighbor.h = neighbor.sqEuclideanDistance
                if(ntype == 5)
                    neighbor.h = neighbor.chebyshevDistance
                if(ntype == 6)
                    if(bAllowDiagonal)
                        neighbor.h = 8 - neighbor.lstNeighbours.length;
                    else
                        neighbor.h = 4 - neighbor.lstNeighbours.length;


                lstFrontier.push(neighbor);
            }
            else if(gScore < neighbor.g) {
                gScoreIsBest = true;
            }

            if(gScoreIsBest) {
                neighbor.parent = currentNode;
                neighbor.g = gScore;
                neighbor.f = neighbor.g + neighbor.h;
            }        
        }
    }
 return [];
}

// add the Neighbours of each gridElement to the dataStructure of the gridElement
function mapNeighbours(g, x, y)
{
    grid = g;
    maxX = x;
    maxY = y;
    for(let i = 0; i < grid.length; i++)
    {
        let column = grid[i];
        for( let j =0; j < column.length; j++)
        {
            if(grid[i][j].bBlock)
                continue;
            let gridElement = column[j];

            if(j - 1 >= 0 && !grid[i][j - 1].bBlock) // left
                gridElement.lstNeighbours.push(grid[i][j - 1]);

            if(i - 1 >= 0 && !grid[i - 1][j].bBlock) //top
                gridElement.lstNeighbours.push(grid[i - 1][j]);

            if(j + 1 <= maxX - 1 && !grid[i ][j + 1].bBlock) //right
                gridElement.lstNeighbours.push(grid[i ][j + 1]);

            if(i + 1 <= maxY - 1 && !grid[i + 1][j].bBlock) //bottom
                gridElement.lstNeighbours.push(grid[i + 1][j]);

            if(bAllowDiagonal)
            {
                if(j - 1 >= 0 && i - 1 >= 0  && !grid[i - 1][j - 1].bBlock) // left top
                gridElement.lstNeighbours.push(grid[i - 1][j - 1]);

            if(i - 1 >= 0 && j + 1 <= maxX - 1 && !grid[i - 1][j + 1].bBlock) // right top
                gridElement.lstNeighbours.push(grid[i - 1][j + 1]);

            if(j - 1 >= 0 && i + 1 <= maxY - 1 && !grid[i + 1][j - 1].bBlock) //bottom left
                gridElement.lstNeighbours.push(grid[i + 1][j - 1]);

            if(i + 1 <= maxY - 1 && j + 1 <= maxX - 1 && !grid[i + 1][j + 1].bBlock) //bottom right
                gridElement.lstNeighbours.push(grid[i + 1][j + 1]);
            }
        }
    }
}

// create Grid Element data structue
function getNewGridElement(j, i, bBlock, ed, sed, mhd, chd,ele)
{
    let obj = {};
    obj.x = j;
    obj.y = i;
    obj.bBlock = bBlock;
    obj.ele = ele;

    obj.manhattanDistance = mhd;
    obj.euclideanDistance = ed;
    obj.sqEuclideanDistance = sed;
    obj.chebyshevDistance = chd;
    obj.lstNeighbours = [];
    return obj;
}

//Self calling function to render and solve the Maze with diffrent heuristics
(function ()
{
let body = document.getElementsByTagName("body")[0];

let mazeWrapper = document.createElement("div");
mazeWrapper.style.position = "absolute";
mazeWrapper.style.left = "50px";
mazeWrapper.style.top = "50px";
mazeWrapper.style.width = "800px";
mazeWrapper.style.height = "500px";


let grid = [];

let column = 8 * scaleFactor;
let rows = 5 * scaleFactor;

maxX = column;
maxY = rows;
for( let i = 0; i < rows; i++)
{
    let row = [];
    for(let j =0; j < column; j++)
    {
        let gridBlock = document.createElement("div");
        gridBlock.style.position = "relative";
        gridBlock.style.left = j + "px";
        gridBlock.style.top = i  + "px";
        gridBlock.style.width =  (100 / scaleFactor) + "px";
        gridBlock.style.height = (100 / scaleFactor) + "px";
        gridBlock.style.display = "inline-block"
        gridBlock.style.backgroundColor = "grey";


        let num = Math.floor(Math.random() * 10);
        let bBlock = false;
        if((i== 0 && j== 0)) //start grid element
        {
            gridBlock.style.backgroundColor = "green";
        }
        else if((i == rows - 1 && j == column - 1)) // goal grid element
        {
            gridBlock.style.backgroundColor = "red";
        }
        else if((i % num != 0 && j % num != 0 ) 
            && Math.random() <= Math.random()) // randomly blocking elements
        {
            gridBlock.style.backgroundColor = "black";
            bBlock = true;
        }
        let manhattanDistance = 0;
        let euclideanDistance = 0;
        let sqEuclideanDistance = 0;
        let chebyshevDistance = 0;

        if(!bBlock)
        {
            manhattanDistance =  computeManhattanDistance(j, i, column - 1,rows - 1);
            euclideanDistance = coumputeEuclideanDistance(j, i, column - 1,rows - 1);
            sqEuclideanDistance = squaredEuclideanDistance(j, i, column - 1,rows - 1);
            chebyshevDistance = computechebyshevDistance(j, i, column - 1,rows - 1);
        }

        //  creates the data structure of the grid element
        let obj = getNewGridElement(j, i, bBlock, euclideanDistance, sqEuclideanDistance,
             manhattanDistance, chebyshevDistance,gridBlock);
        row.push(obj);
        mazeWrapper.appendChild(gridBlock);
    }
    grid.push(row); //filling the data in the Grid data structure
}
mapNeighbours(grid, maxX, maxY);
 
 aStartByH0();
 aStartByManhattan();
 aStartByEuclidean();
 aStartBySquaredEuclidean();
 aStartBychebyshev();
 aStartByFreedomOfMovement();

body.appendChild(mazeWrapper);
})();
