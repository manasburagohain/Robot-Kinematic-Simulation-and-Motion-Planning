/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    neighbours = [[-1,0],[1,0],[0,1],[0,-1]]

    counter = 0;
    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            if(G[iind][jind].x + (eps/2) >= q_init[0] && G[iind][jind].x - (eps/2) <= q_init[0] && G[iind][jind].y + (eps/2) >= q_init[1] && G[iind][jind].y - (eps/2) <= q_init[1]){
                G[iind][jind].distance = 0;
                if(search_alg === "breadth-first")
                    G[iind][jind].priority = counter;
                else
                    G[iind][jind].priority = G[iind][jind].distance + heuristic(G[iind][jind].x, G[iind][jind].y);
                G[iind][jind].queued = true;
                if(search_alg === "depth-first")
                    stack_insert(visit_queue, G[iind][jind]);
                else
                    minheap_insert(visit_queue, G[iind][jind]);
            }

        }
    }
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    if(visit_queue.length === 0)
        return "failed";

    curr_node = minheap_extract(visit_queue);
    curr_node.queued = false;
    curr_node.visited = true;
    search_visited+=1;
    draw_2D_configuration([curr_node.x,curr_node.y], "visited")
    collision = testCollision([curr_node.x, curr_node.y])

    if(collision === true)
        return "failed";
    
    if (curr_node.x + (eps/2) >= q_goal[0] && curr_node.x - (eps/2) < q_goal[0] && curr_node.y + (eps/2) >= q_goal[1] && curr_node.y - (eps/2) < q_goal[1]) {
       drawHighlightedPathGraph(curr_node);
       search_iterate = false;
       return "succeeded"; 
    }

    for(ind = 0; ind < neighbours.length; ind++){
        neigh = G[curr_node.i + neighbours[ind][0]][curr_node.j + neighbours[ind][1]]
        if(neigh.visited === false && testCollision([neigh.x, neigh.y]) === false){
            if(neigh.distance > G[curr_node.i][curr_node.j].distance + eps){
                neigh.parent = curr_node;
                neigh.distance = G[curr_node.i][curr_node.j].distance + eps;
                neigh.priority = neigh.distance + heuristic(neigh.x, neigh.y); 
                draw_2D_configuration([neigh.x, neigh.y], "queued")
                minheap_insert(visit_queue, neigh);
            }
        }
    }
    

    return "iterating";


}

function iterateGreedySearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    if(visit_queue.length === 0)
        return "failed";

    curr_node = minheap_extract(visit_queue);
    curr_node.queued = false;
    curr_node.visited = true;
    search_visited+=1;
    draw_2D_configuration([curr_node.x,curr_node.y], "visited")
    collision = testCollision([curr_node.x, curr_node.y])

    if(collision === true)
        return "failed";
    
    if (curr_node.x + (eps/2) >= q_goal[0] && curr_node.x - (eps/2) < q_goal[0] && curr_node.y + (eps/2) >= q_goal[1] && curr_node.y - (eps/2) < q_goal[1]) {
       drawHighlightedPathGraph(curr_node);
       search_iterate = false;
       return "succeeded"; 
    }

    for(ind = 0; ind < neighbours.length; ind++){
        neigh = G[curr_node.i + neighbours[ind][0]][curr_node.j + neighbours[ind][1]]
        if(neigh.visited === false && neigh.queued === false && testCollision([neigh.x, neigh.y]) === false){
            if(neigh.distance > G[curr_node.i][curr_node.j].distance + eps){
                neigh.parent = curr_node;
                neigh.distance = G[curr_node.i][curr_node.j].distance + eps;
                neigh.priority = heuristic(neigh.x, neigh.y);
                neigh.queued = true; 
                draw_2D_configuration([neigh.x, neigh.y], "queued")
                minheap_insert(visit_queue, neigh);
            }
        }
    }
    

    return "iterating";


}

function iterateBreadthSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    if(visit_queue.length === 0)
        return "failed";

    curr_node = minheap_extract(visit_queue);
    curr_node.queued = false;
    curr_node.visited = true;
    search_visited+=1;
    draw_2D_configuration([curr_node.x,curr_node.y], "visited")
    collision = testCollision([curr_node.x, curr_node.y])

    if(collision === true)
        return "failed";
    
    if (curr_node.x + (eps/2) >= q_goal[0] && curr_node.x - (eps/2) <= q_goal[0] && curr_node.y + (eps/2) >= q_goal[1] && curr_node.y - (eps/2) <= q_goal[1]) {
       drawHighlightedPathGraph(curr_node);
       search_iterate = false;
       return "succeeded"; 
    }

    for(ind = 0; ind < neighbours.length; ind++){
        neigh = G[curr_node.i + neighbours[ind][0]][curr_node.j + neighbours[ind][1]]
        if(neigh.visited === false && testCollision([neigh.x, neigh.y]) === false){
            if(neigh.distance > G[curr_node.i][curr_node.j].distance + eps){
                neigh.parent = curr_node;
                neigh.distance = G[curr_node.i][curr_node.j].distance + eps;
                neigh.priority = ++counter;
                draw_2D_configuration([neigh.x, neigh.y], "queued")
                minheap_insert(visit_queue, neigh);
            }
        }
    }
    

    return "iterating";


}

function iterateDepthSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    if(visit_queue.length === 0)
        return "failed";

    curr_node = stack_pop(visit_queue);
    curr_node.queued = false;
    curr_node.visited = true;
    search_visited+=1;
    draw_2D_configuration([curr_node.x,curr_node.y], "visited")
    collision = testCollision([curr_node.x, curr_node.y])

    if(collision === true)
        return "failed";
    
    if (curr_node.x + (eps/2) >= q_goal[0] && curr_node.x - (eps/2) <= q_goal[0] && curr_node.y + (eps/2) >= q_goal[1] && curr_node.y - (eps/2) <= q_goal[1]) {
       drawHighlightedPathGraph(curr_node);
       search_iterate = false;
       return "succeeded"; 
    }

    for(ind = 0; ind < neighbours.length; ind++){
        neigh = G[curr_node.i + neighbours[ind][0]][curr_node.j + neighbours[ind][1]]
        if(neigh.visited === false && testCollision([neigh.x, neigh.y]) === false){
            if(neigh.distance > G[curr_node.i][curr_node.j].distance + eps){
                neigh.parent = curr_node;
                neigh.distance = G[curr_node.i][curr_node.j].distance + eps;
                draw_2D_configuration([neigh.x, neigh.y], "queued")
                stack_insert(visit_queue, neigh);
            }
        }
    }
    

    return "iterating";


}
//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.
function heuristic(x, y) {
    return Math.sqrt(Math.pow((q_goal[0]-x),2)+Math.pow((q_goal[1]-y),2));
}

function stack_insert(stack, new_element){
    stack.push(new_element);
}

function stack_pop(stack){
    return stack.splice(stack.length-1);
}
    

function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element)

    if (heap.length > 1) {
            let current = heap.length - 1

            /* Traversing up the parent node until the current node (current) is greater than the parent (current/2)*/
            while (current > 0 && heap[Math.floor((current-1)/2)].priority > heap[current].priority) {

                /* Swapping the two nodes by using the ES6 destructuring syntax*/
                [heap[Math.floor((current-1)/2)], heap[current]] = [heap[current], heap[Math.floor((current-1)/2)]]
                current = Math.floor((current-1)/2)
            }
    }
}

function minheapify(heap , idx)
{
  let l = 2*idx + 1
  let r = 2*idx + 2
  let smallest = idx

  if(l < heap.length && heap[l].priority < heap[smallest].priority){
    smallest = l
  }
  if(r < heap.length && heap[r].priority < heap[smallest].priority){
    smallest = r
  }

  if(smallest != idx){
    [heap[idx], heap[smallest]] = [heap[smallest], heap[idx]]
    minheapify(heap, smallest)
  }
}

function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
  let smallest = heap[0]
  heap[0] = heap[heap.length - 1]
  heap.pop()
  minheapify(heap, 0)

  return smallest
}