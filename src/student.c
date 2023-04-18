/*
 * student.c
 * Multithreaded OS Simulation for CS 2200 and ECE 3058
 *
 * This file contains the CPU scheduler for the simulation.
 */

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "os-sim.h"

#define DEBUG 0

/** Function prototypes **/
extern void idle(unsigned int cpu_id);
extern void preempt(unsigned int cpu_id);
extern void yield(unsigned int cpu_id);
extern void terminate(unsigned int cpu_id);
extern void wake_up(pcb_t *process);
static pcb_t* pop_from_queue();
static void push_to_queue(pcb_t *pcb);
int strcmp(const char*, const  char*);



/*
 * current[] is an array of pointers to the currently running processes.
 * There is one array element corresponding to each CPU in the simulation.
 *
 * current[] should be updated by schedule() each time a process is scheduled
 * on a CPU.  Since the current[] array is accessed by multiple threads, you
 * will need to use a mutex to protect it.  current_mutex has been provided
 * for your use.
 */
static pcb_t **current;
static pthread_mutex_t current_mutex;

/*
 * Node for the linked list
 */
typedef struct _Node {
    struct _Node* next;
    pcb_t* data;
} Node;

/*
 *Global static varialbles
*/
//LinkedList Variables
static Node* process_linked_list = NULL;
static pthread_mutex_t process_linked_list_mutex;
static pthread_cond_t process_linked_list_not_idle;

//Others
static int LRTF = 0;
static int alg = 0;//0- FIFO, 1-RR, 2-LTRF
static int empty = 1;
static int time_slice = -1;
static unsigned int cpu_count;

//Create a new node
static Node* createNode() {
    Node* newNode = (Node*) malloc(sizeof(Node));
    return newNode;
}

/*Push process to linked list*/
static void push_to_queue(pcb_t* data) {
    //lock process linked list
    pthread_mutex_lock(&process_linked_list_mutex);
    if (DEBUG) {
        printf("Pushing to Queue \n");
    }
    //Create Node for current process

    Node* node = createNode();
    node->data = data;
    node->next = NULL;
    //pointer to "head" essentially
    Node* curr = process_linked_list;
    if (curr && LRTF) {
        // Apply LRTF
        while (curr->next && curr->next->data->time_remaining > data->time_remaining) {
            curr = curr->next;
        }
    } else if (curr && !LRTF) {
        //else FIFO or RoundRobin
        while (curr->next) { 
            curr = curr->next;
        }
    }
    if (curr) {
        node->next = curr->next;
        curr->next = node;
    } else {
        process_linked_list = node;
    }
    empty = 0;
    if (DEBUG) {
        printf("DONE\n");
    }
    //unlock mutexes notify all cpus that process list now contains info to use
    pthread_cond_signal(&process_linked_list_not_idle);
    pthread_mutex_unlock(&process_linked_list_mutex);
}

//removes most desired noe (at front)
static pcb_t* pop_from_queue() {
    //lock
    pthread_mutex_lock(&process_linked_list_mutex);
    if (DEBUG) {
        printf("Pop from Queue \n");
    }
    pcb_t* result = NULL;
    //pop top value from process_linked_list given there are values in the queue
    if (process_linked_list) {
        result = process_linked_list->data;
        Node* result_node = process_linked_list;
        process_linked_list = process_linked_list->next;
        if (!process_linked_list) empty = 1;
        //free allocated node
        free(result_node);
    }
    //unlock
    pthread_mutex_unlock(&process_linked_list_mutex);
    return result;
}


/*
 * schedule() is your CPU scheduler.  It should perform the following tasks:
 *
 *   1. Select and remove a runnable process from your ready queue which 
 *	you will have to implement with a linked list or something of the sort.
 *
 *   2. Set the process state to RUNNING
 *
 *   3. Call context_switch(), to tell the simulator which process to execute
 *      next on the CPU.  If no process is runnable, call context_switch()
 *      with a pointer to NULL to select the idle process.
 *	The current array (see above) is how you access the currently running process indexed by the cpu id. 
 *	See above for full description.
 *	context_switch() is prototyped in os-sim.h. Look there for more information 
 *	about it and its parameters.
 */
static void schedule(unsigned int cpu_id)
{
    if (DEBUG) {
        printf("START SCHEDULE \n");
    }
    
    //Select and remove a runnable process from your ready queue which 
    //you will have to implement with a linked list or something of the sort.
    //Implement with queue
    pcb_t* process = pop_from_queue();
    
    //Set the process state to RUNNING
    if (process != NULL) {
        process->state = PROCESS_RUNNING;
    }

    //lock, edit, unlock current
    pthread_mutex_lock(&current_mutex);
    current[cpu_id] = process;
    pthread_mutex_unlock(&current_mutex);

    //call for context switch
    context_switch(cpu_id, process, time_slice);
}


/*
 * idle() is your idle process.  It is called by the simulator when the idle
 * process is scheduled.
 *
 * This function should block until a process is added to your ready queue.
 * It should then call schedule() to select the process to run on the CPU.
 */
extern void idle(unsigned int cpu_id)
{
    /* FIX ME */
    //lock process until a process is added to your ready queue
    pthread_mutex_lock(&process_linked_list_mutex);
    if (DEBUG) {
        printf("IDLE\n");
    }
    //If linked list empty
    if (empty) {
        pthread_cond_wait(&process_linked_list_not_idle, &process_linked_list_mutex);
    }
    
    //unlock and schedule new process to execute
    pthread_mutex_unlock(&process_linked_list_mutex);
    schedule(cpu_id);

    if (DEBUG) {
        printf("END IDLE\n");
    }
}


/*
 * preempt() is the handler called by the simulator when a process is
 * preempted due to its timeslice expiring.
 *
 * This function should place the currently running process back in the
 * ready queue, and call schedule() to select a new runnable process.
 */
extern void preempt(unsigned int cpu_id)
{
    /* FIX ME */
    //lock current, gain state, unlock current
    pthread_mutex_lock(&current_mutex);
    pcb_t* curr = current[cpu_id];
    

    //set ready, push_to_queue, and schedule
    curr->state = PROCESS_READY;
    pthread_mutex_unlock(&current_mutex);
    push_to_queue(curr);
    schedule(cpu_id);
}


/*
 * yield() is the handler called by the simulator when a process yields the
 * CPU to perform an I/O request.
 *
 * It should mark the process as WAITING, then call schedule() to select
 * a new process for the CPU.
 */
extern void yield(unsigned int cpu_id)
{
    /* FIX ME */
    if (DEBUG) {
        printf("YIELD %d\n", cpu_id);
    }

    //lock current
    pcb_t *curr;
    pthread_mutex_lock(&current_mutex);
    curr = current[cpu_id];
    curr->state = PROCESS_WAITING;
    pthread_mutex_unlock(&current_mutex);

    //schedule the next process at cpu_id
    schedule(cpu_id);
    if (DEBUG) {
        printf("END YIELD\n");
    }
}


/*
 * terminate() is the handler called by the simulator when a process completes.
 * It should mark the process as terminated, then call schedule() to select
 * a new process for the CPU.
 */
extern void terminate(unsigned int cpu_id)
{
    /* FIX ME */
    if (DEBUG) {
        printf("TERMINATE\n");
    }
    pcb_t *curr;
    //lock current, edit state, unlock current
    pthread_mutex_lock(&current_mutex);
    curr = current[cpu_id];
    curr->state = PROCESS_TERMINATED;
    pthread_mutex_unlock(&current_mutex);

    //schedule the next process at cpu_id
    schedule(cpu_id);
}


/*
 * wake_up() is the handler called by the simulator when a process's I/O
 * request completes.  It should perform the following tasks:
 *
 *   1. Mark the process as READY, and insert it into the ready queue.
 *
 *   2. If the scheduling algorithm is LRTF, wake_up() may need
 *      to preempt the CPU with lower remaining time left to allow it to
 *      execute the process which just woke up with higher reimaing time.
 * 	However, if any CPU is currently running idle,
* 	or all of the CPUs are running processes
 *      with a higher remaining time left than the one which just woke up, wake_up()
 *      should not preempt any CPUs.
 *	To preempt a process, use force_preempt(). Look in os-sim.h for 
 * 	its prototype and the parameters it takes in.
 */
extern void wake_up(pcb_t *process)
{
    /* FIX ME */
    if (DEBUG) {
        printf("WAKE UP ");
    }
    unsigned int preempt_id;
    unsigned int cause_idle;
    unsigned int cause_preempt;
    //update process state and push_to_queue onto queue
    //pthread_mutex_lock(&current_mutex);
    process->state = PROCESS_READY;
    //pthread_mutex_unlock(&current_mutex);

    push_to_queue(process);

    //if LRTF
    if (LRTF) {
        pthread_mutex_lock(&current_mutex);
        //variables
        preempt_id = 0;
        cause_idle = 0;
        cause_preempt = 0;

        //loop through all cpus and find an idle CPU
        for (unsigned int i = 0; i < cpu_count; i++) {
            // Found an idle cpu so break (no need to force preempt)
            if (!current[i]) {
                cause_idle = 1;
                break;
                //No need to force idle CPU
            } 
            //if not, update index of lowest time remaining 
            if (current[i] -> time_remaining < process -> time_remaining) {
                cause_preempt = 1;
            }
            if (current[i]->time_remaining < current[preempt_id]->time_remaining) {
                preempt_id = i;
            }
        }
        //unlock
        pthread_mutex_unlock(&current_mutex);
        //if spot free and preempt high, call force preempt
        if (!cause_idle && cause_preempt) {
            force_preempt(preempt_id);
        }
    }
    if (DEBUG) {
        printf("WOKEN UP\n");
    }
}


/*
 * main() simply parses command line arguments, then calls start_simulator().
 * You will need to modify it to support the -l and -r command-line parameters.
 */
int main(int argc, char *argv[])
{
    if (argc < 2 || argc > 5) {
        fprintf(stderr, "Multithreaded OS Simulator\n"
            "Usage: ./os-sim <# CPUs> [ -l | -r <time slice> ]\n"
            "    Default : FIFO Scheduler\n"
            "         -l : Longest Remaining Time First Scheduler\n"
            "         -r : Round-Robin Scheduler\n\n");
        return -1;
    }
    

    /* FIX ME - Add support for -l and -r parameters*/
    // Get the type and the timeslice value
 

    for(int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-r") == 0 && i < argc - 1) {
            alg = 1; // RR
            time_slice = atoi(argv[i + 1]);
        }
        else if(argc == 3 && strcmp(argv[i], "-l") == 0) {
            alg = 2; // LTRF
            LRTF = 1;
            time_slice = -1;
        }
    }
    cpu_count = strtoul(argv[1], NULL, 0);
    printf("%d\n", alg);
    
    

    /* Allocate the current[] array and its mutex */
    current = malloc(sizeof(pcb_t*) * cpu_count);
    for (unsigned int i = 0; i < cpu_count; i++) {
        current[i] = NULL;
    }
    assert(current != NULL);
    pthread_mutex_init(&current_mutex, NULL);
    pthread_mutex_init(&process_linked_list_mutex, NULL);
    pthread_cond_init(&process_linked_list_not_idle, NULL);

    /* Start the simulator in the library */
    start_simulator(cpu_count);
    

    pthread_mutex_destroy(&current_mutex);
    pthread_mutex_destroy(&process_linked_list_mutex);
    pthread_cond_destroy(&process_linked_list_not_idle);
    

    return 0;
}


