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

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    let smallest = heap[1]

        /* When there are more than two elements in the array, we put the right most element at the first position
            and start comparing nodes with the child nodes
        */
        if (heap.length > 2) {
            heap[1] = heap[heap.length-1]
            heap.splice(heap.length - 1)

            if (heap.length === 3) {
                if (heap[1] > heap[2]) {
                    [heap[1], heap[2]] = [heap[2], heap[1]]
                }
                return smallest
            }

            let current = 1
            let leftChildIndex = current * 2
            let rightChildIndex = current * 2 + 1

            while (heap[leftChildIndex] &&
                    heap[rightChildIndex] &&
                    (heap[current] < heap[leftChildIndex] ||
                        heap[current] < heap[rightChildIndex])) {
                if (heap[leftChildIndex] < heap[rightChildIndex]) {
                    [heap[current], heap[leftChildIndex]] = [heap[leftChildIndex], heap[current]]
                    current = leftChildIndex
                } else {
                    [heap[current], heap[rightChildIndex]] = [heap[rightChildIndex], heap[current]]
                    current = rightChildIndex
                }

                leftChildIndex = current * 2
                rightChildIndex = current * 2 + 1
            }
        }

        /* If there are only two elements in the array, we directly splice out the first element */

        else if (heap.length === 2) {
            heap.splice(1, 1)
        } else {
            return null
        }

        return smallest
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;





