/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element)

    if (heap.length > 1) {
            let current = heap.length - 1

            /* Traversing up the parent node until the current node (current) is greater than the parent (current/2)*/
            while (current > 0 && heap[Math.floor((current-1)/2)] > heap[current]) {

                /* Swapping the two nodes by using the ES6 destructuring syntax*/
                [heap[Math.floor((current-1)/2)], heap[current]] = [heap[current], heap[Math.floor((current-1)/2)]]
                current = Math.floor((current-1)/2)
            }
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

function minheapify(heap , ind)
{
  let l = 2*ind + 1
  let r = 2*ind + 2
  let smallest = ind

  if(l < heap.length && heap[l] < heap[smallest]){
    smallest = l
  }
  if(r < heap.length && heap[r] < heap[smallest]){
    smallest = r
  }

  if(smallest != ind){
    [heap[ind], heap[smallest]] = [heap[smallest], heap[ind]]
    minheapify(heap, smallest)
  }
}
// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
  let smallest = heap[0]
  heap[0] = heap[heap.length - 1]
  heap.length--
  minheapify(heap, 0)

  return smallest
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;





