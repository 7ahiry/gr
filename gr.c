/**
 *  \file   gradient.c
 *  \brief  module for Gradient Routing protocol
 *  \author Tahiry Razafindralambo
 *  \date   2008
 **/
#include <stdio.h>
#include <include/modelutils.h>
#include <include/types.h>
//#define DEBUG_T
#define STATS

/* ************************************************** 
INFO:
- Implementation of a gradient ruting protocol. 
- A gradient is the distance of each sensor in number of hops to the sink
- Node 0 (c->node == 0) is the sink, it initiate the creation of the gradient
- The gradient of depth is 0 for the sink

MESSAGE TYPE:
- BUILD : gradient construction message
    - The sink sends the build message at regular interval (with a sequence number)
    - Contains:
        - Source
        - sequence number
        - depth of the sender
        - ...
    - If a node receives a BUILD message, 
        - it changes its status to NODE_ON
        - it updates its depth (msg.depth ++) 
            - to be done in an intelligent way
        - Sends the message (local broadcast) 
            - avoid broadcast strom
- DATA : Data message
    - For now only node 1 can send data messages 
        - local broadcast
            - Can be more intelligent
        - with its id, a time stamp (compute delay), depth of the node, origin depth, sequence number...
    - If a node receives a data message
        - it forwards it
            - In an intelligent way
            - check sequence number, ... 
    - If the sink receive the message : WE ARE DONE
        - print some stats about the messages

TODO : 
    - building the gradient
    - sending message to the sink
    - statistics constructions + visualisation


************************************************** */




model_t model =  {
    "Gradient Routing",
    "Tahiry Razafindralambo",
    "0.1",
    MODELTYPE_APPLICATION, 
    {NULL, 0}
};
#define SENSOR  0
#define SINK    1

#define STATIC  0
#define MOVING  1

#define BUILD 0
#define DATA  1

#define NODE_OFF 0
#define NODE_ON 1

#define MES_NO -1
#define MES_BU 0

#define BUFFER 10
#define SEQUENCE 10


/* ************************************************** */
/* *************** STRUCTURE DEFINITION ************* */
/* ************************************************** */
/* Common entity data */
struct entitydata{
    uint64_t Delay; // short delay before sending a message
    uint64_t Period;
    uint64_t Jitter;
    uint64_t TimeSpace;

    int packet_seq;
};

/* Data Packet header */
struct packet_header {
    int       p_src;
    int       p_dst;
    int       p_type;
    int       p_seqno;
    int       p_depth;
    int       p_origin;
    int       p_status;
    double    p_pos_x;
    double    p_pos_y;
    uint64_t  p_stamp;
};

/* Private node data */
struct _node_private {
    int seqno;
    int depth;
    int from;
    int type;
    int status;
    int msg_status;
    int node_status;
    int *overhead;
    struct packet_header p[BUFFER];
    int seq[SEQUENCE];
    int buffer_pointer;
    int seq_pointer;

    int no_packet_sent;
    int no_packet_recv;
    int no_packet_drop;

    double distance ;
    double speed    ;
    uint64_t timestamp ;
    double cosx ;
    double sinx ;
};

/* ************************************************** */
/* ************************************************** */
int tx_build(call_t *c, void *args);
int tx_data(call_t *c, void *args);
int tx_forward(call_t *c, void *args);
int move(call_t *c, void *args);
int my_energy(call_t *c, void *args);
void add_seq(call_t *c, int s);
int check_seq(call_t *c, int s);
int updateposition(call_t *c);
double d(int i, int j);
double dpos(int x_1, int y_1, int x_2, int y_2);

/* ************************************************** */
/* ************************************************** */

double d(i,j){
    // Compte the distance between two nodes based on their ID
    // return the distance
    position_t *p_i, *p_j;
    double d;
    p_i = get_node_position(i);
    p_j = get_node_position(j);
    
    d = sqrt(  (p_i->x-p_j->x)*(p_i->x-p_j->x) + (p_i->y-p_j->y)*(p_i->y-p_j->y)  );
    return d;
}

double dpos(x_1,y_1,x_2,y_2){
    // Compte the distance between two nodes based on their (x,y) position
    // return the distance
    double d;
    
    d = sqrt(  (x_1 - x_2)*(x_1 - x_2) + (y_1 - y_2)*(y_1 - y_2)  );
    return d;
}

/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {
    // Initialisation of the gradient
    // All variable that are to be common for all nodes (at the gradient level should be here)
    // Some times are defined here and can be change in the xml files
    // All the variables are store in the entitydata structure (define above)
    struct entitydata *entitydata = malloc(sizeof(struct entitydata));
    param_t *param;

    /* default entity variables */
    entitydata->Delay      = 500000000;    // 0.5s
    entitydata->Period     = 10000000000;  // 10s
    entitydata->Jitter     = 50000000;     // 0.05s
    entitydata->TimeSpace  = 1000000000;   // 1s
    entitydata->packet_seq = 0;

    /* reading the "init" markup from the xml config file */
    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "Delay")) {
            if (get_param_time(param->value, &(entitydata->Delay))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "Period")) {
            if (get_param_time(param->value, &(entitydata->Period))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "Jitter")) {
            if (get_param_time(param->value, &(entitydata->Jitter))) {
                goto error;
            }
        }     
        if (!strcmp(param->key, "TimeSpace")) {
            if (get_param_time(param->value, &(entitydata->TimeSpace))) {
                goto error;
            }
        }    
    } 
    set_entity_private_data(c, entitydata);
    return 0;

    error:
        free(entitydata);
        return -1;
}

int destroy(call_t *c) {
    // destroying the entitydata structure 
    // can be usefull to put some statistics here (end of simulation)
    return 0;
}

/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    // Initialisation of the gradient at a node level
    // create the local variable of a node
    // All variable for each node is stored in  _node_private structure (see above)
    struct _node_private *nodedata = malloc(sizeof(struct _node_private));
    //struct entitydata *entitydata = get_entity_private_data(c);
    int i = get_entity_links_down_nbr(c);
    param_t *param;

    /* default values */
    nodedata->seqno = -1;
    nodedata->depth = -1;
    nodedata->from = -1;
    nodedata->status = STATIC;
    nodedata->msg_status = MES_NO;
    nodedata->type = SENSOR;
    nodedata->node_status = NODE_OFF;
    nodedata->buffer_pointer = 0;
    nodedata->seq_pointer    = 0;
    nodedata->no_packet_sent = 0;
    nodedata->no_packet_recv = 0;

    nodedata->distance = 0;
    nodedata->speed    = 0;
    nodedata->timestamp = 0;
    nodedata->cosx = 0;
    nodedata->sinx = 0;

    /* get parameters */
    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "type")) {
            if (get_param_integer(param->value, &(nodedata->type))) {
                goto error;
            }
        }
    }
    
    /*define node 0 as the sink, this can be decided by type in the xml file
     * putting <default type="1"/>
     */
    if (c->node == 0){
        nodedata->type = SINK;
    }

    /* alloc overhead memory */
    if (i) { nodedata->overhead = malloc(sizeof(int) * i); } 
    else   { nodedata->overhead = NULL; }

    set_node_private_data(c, nodedata);
    return 0;

    error:
        free(nodedata);
        return -1;
}

int unsetnode(call_t *c) {
    // Function call when de-allocating the node and especially its _node_private structure
    // This function is also called when a node dies (battery)
    // usefull for individual statistics at the end of the simulation
    struct _node_private *nodedata = get_node_private_data(c);
    //struct entitydata *entitydata = get_entity_private_data(c); 
    // print node stat before exit here !
    #ifdef DEBUG_T 
        position_t *position;
        position = get_node_position(c->node);
        printf("(%03i) d-%03i p-%03i \t posx = %04.3lf - posy = %lf - posz = %lf \t \n",
                c->node,nodedata->depth,nodedata->from,position->x,position->y,position->z);
    #endif    
    #ifdef STATS
        printf("(%i) %i %i %i %i\n", 
                c->node,nodedata->depth,
                nodedata->no_packet_sent,nodedata->no_packet_recv,nodedata->no_packet_drop); 
    #endif    

    if (nodedata->overhead) {
        free(nodedata->overhead);
    }
    free(nodedata);
    return 0;
}

/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
    // Bootstraping the simulation
    // First actions perform by a node go here
    // for example sending the first BUILD message for the sink
    // or sending the first data message for the sensors
    // make use of scheduler call back (use and abuse - not that much, since its slows simulation-)
    struct _node_private *nodedata = get_node_private_data(c);
    int i = get_entity_links_down_nbr(c);
    entityid_t *down = get_entity_links_down(c);
    struct entitydata *entitydata = get_entity_private_data(c); 
    
    /* get overhead */
    while (i--) {
        call_t c0 = {down[i], c->node, c->entity};
        nodedata->overhead[i] = GET_HEADER_SIZE(&c0);
    }
    
    /* eventually schedule callback */
    if (nodedata->type == SINK) { // the sink part 
      nodedata->seqno = 0;
      nodedata->from = c->node;
      nodedata->depth = 0;
      nodedata->node_status = NODE_ON;
      scheduler_add_callback(get_time() + 0, c, tx_build, NULL);
    } else { 
        // all other nodes except the sink
        // schedule data transmission
        if (c->node == 1 ){
            scheduler_add_callback(get_time() + 
                             entitydata->Period + 
                             get_random_time_range(0,entitydata->Jitter) + 
                             get_random_time_range(0,entitydata->TimeSpace), 
                             c, tx_data, NULL);
        }
    }
    return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
    // useless for us here
    return 0;  
}

int updateposition(call_t *c){
    // useless for us here
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int tx_build(call_t *c, void *args) {
    // transmitting build message
    struct _node_private *nodedata = get_node_private_data(c);
    struct entitydata *entitydata = get_entity_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
    packet_t *packet = packet_alloc(c, nodedata->overhead[0] + sizeof(struct packet_header) );
    struct packet_header *header = (struct packet_header *) (packet->data + nodedata->overhead[0]);


    /* set mac header */
    if (SET_HEADER(&c0, packet, &destination) == -1) {
        packet_dealloc(packet);
        return -1;
    }
    header->p_src = c->node;
    header->p_dst = -1;
    header->p_type = BUILD;
    header->p_seqno = nodedata->seqno; 
    header->p_depth = nodedata->depth;
    header->p_stamp = get_time();
    header->p_origin = c->node;
    header->p_pos_x = get_node_position(c->node)->x;
    header->p_pos_y = get_node_position(c->node)->y;
    header->p_status = nodedata->status;
    /* can schedule build message again*/
    nodedata->msg_status = MES_NO;
#ifdef DEBUG_T    
    printf("%lli (%03i) \t d-%3i\n", get_time(),c->node,nodedata->depth);
#endif

    TX(&c0, packet);
    
    if (nodedata->type == SINK) {
        nodedata->seqno ++;
        // reschedule gradient build after 10*entitydata->Period
        scheduler_add_callback(get_time() + 10*entitydata->Period, c, tx_build, NULL);
    }
    return 1;
}

int tx_data(call_t *c, void *args) {
    // transmitting data messages
    struct _node_private *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
    packet_t *packet = packet_alloc(c, nodedata->overhead[0] + sizeof(struct packet_header) );
    struct packet_header *header = (struct packet_header *) (packet->data + nodedata->overhead[0]);
    struct entitydata *entitydata = get_entity_private_data(c);

    if ( nodedata->node_status != NODE_ON ){
        scheduler_add_callback(get_time() + 
                                entitydata->Period + get_random_time_range(0,entitydata->Jitter), 
                                c, tx_data, NULL);    
        return 1;
    }



    /* set mac header */
    if (SET_HEADER(&c0, packet, &destination) == -1) {
        packet_dealloc(packet);
        return -1;
    } 

    header->p_src = c->node;
    header->p_dst = -1;
    header->p_type = DATA;
    header->p_depth = nodedata->depth;
    header->p_stamp = get_time();
    header->p_origin = c->node;
    header->p_pos_x = get_node_position(c->node)->x;
    header->p_pos_y = get_node_position(c->node)->y;
    header->p_status = nodedata->status;
    nodedata->no_packet_sent ++;
    entitydata->packet_seq ++;
    header->p_seqno =  entitydata->packet_seq; 

    #ifdef DEBUG_T   
        printf("%lli (%03i) \t d-%3i \t s-%6i r-%6i\n", 
                get_time(),c->node,nodedata->depth,
                nodedata->no_packet_sent,nodedata->no_packet_recv);  
    #endif

    TX(&c0, packet);
    scheduler_add_callback(get_time() + 
                            entitydata->Period + get_random_time_range(0,entitydata->Jitter), 
                            c, tx_data, NULL);
    return 1;
}

int tx_forward(call_t *c, void *args) {
    // forwarding other nodes' messages
    struct _node_private *nodedata = get_node_private_data(c);
    nodedata->buffer_pointer -- ;
    if ( nodedata->buffer_pointer < 0 ) {
        nodedata->buffer_pointer = 0;
        return -1;
    }

    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
    packet_t *packet = packet_alloc(c, nodedata->overhead[0] + sizeof(struct packet_header) );
    struct packet_header *header = (struct packet_header *) (packet->data + nodedata->overhead[0]);
    struct entitydata *entitydata = get_entity_private_data(c);

    /* set mac header */
    if (SET_HEADER(&c0, packet, &destination) == -1) {
        packet_dealloc(packet);
        return -1;
    } 

    header->p_src     = c->node ;
    header->p_dst     = -1 ;
    header->p_type    = DATA ;
    header->p_depth   = nodedata->depth ;
    header->p_seqno   = (nodedata->p[0]).p_seqno ; 
    header->p_origin  = (nodedata->p[0]).p_origin ;
    header->p_pos_x   = (nodedata->p[0]).p_pos_x;
    header->p_pos_y   = (nodedata->p[0]).p_pos_y;
    header->p_status  = nodedata->status ;
    header->p_stamp   = (nodedata->p[0]).p_stamp ;

    /*
    * FIFO implementation
    * */
    if ( nodedata->buffer_pointer > 0 ) {
        int i;
        for (i = 0 ; i < nodedata->buffer_pointer ; i++ ) {
            nodedata->p[i].p_src     = nodedata->p[i+1].p_src    ;
            nodedata->p[i].p_dst     = nodedata->p[i+1].p_dst    ;
            nodedata->p[i].p_type    = nodedata->p[i+1].p_type   ;
            nodedata->p[i].p_seqno   = nodedata->p[i+1].p_seqno  ;
            nodedata->p[i].p_depth   = nodedata->p[i+1].p_depth  ;
            nodedata->p[i].p_origin  = nodedata->p[i+1].p_origin ;
            nodedata->p[i].p_status  = nodedata->p[i+1].p_status ;
            nodedata->p[i].p_stamp   = nodedata->p[i+1].p_stamp  ;
            nodedata->p[i].p_pos_x   = nodedata->p[i+1].p_pos_x  ;
            nodedata->p[i].p_pos_y   = nodedata->p[i+1].p_pos_y  ;
        }
    }


    #ifdef DEBUG_T
        printf("%lli (%03i) \t d-%3i \t %i %i\n", 
                get_time(),c->node,nodedata->depth,nodedata->no_packet_sent,header->p_seqno);  
    #endif

    if ( nodedata->buffer_pointer > 0 ) {
        scheduler_add_callback(get_time() + get_random_time_range(0,entitydata->Jitter), c, tx_forward, NULL);
    }
    TX(&c0, packet);
    return 1;
}

int move(call_t *c, void *args) {
    // useless for us here
    return 0;
}

int my_energy(call_t *c, void *args) {
    entityid_t energy_id = get_energy_entity(c);
    if (c->node == 0){
        return 100;
    } else {
        call_t c1 = {energy_id,c->node,c->entity};
        return IOCTL(&c1,0,0,0);
    }
}

/* ************************************************** */
/* ************************************************** */

void add_seq(call_t *c, int s) {
    // add a sequence number of a packet in a table (table as a finite size)
    // sequence number are globally unique
    // for data packet
    struct _node_private *nodedata = get_node_private_data(c);
    nodedata->seq_pointer ++ ;
    if ( nodedata->seq_pointer >= SEQUENCE ) { 
        nodedata->seq_pointer = nodedata->seq_pointer % (SEQUENCE);
    }
    nodedata->seq[nodedata->seq_pointer] = s;
}

int check_seq(call_t *c, int s) {
    // check if a sequence number is in the table (table as a finite size)
    // for data packet
    struct _node_private *nodedata = get_node_private_data(c);
    int i, helper = 0;
    for ( i = 0 ; i < SEQUENCE ; i ++ ){
        if ( nodedata->seq[i] == s ) {
            helper ++;
        }
    }

    if ( helper > 0 ) 
        return -1;
    else
        return 1;
}


/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {
    // reception of message from lower level
    // This function is call when other layer of the protocol stack when a packet should be sent
    // to the gradient (application layer)
    // All received packets first "arrive in this function"
    int helper = 0;
    struct _node_private *nodedata = get_node_private_data(c);
    struct entitydata *entitydata = get_entity_private_data(c);
    struct packet_header *header = (struct packet_header *) (packet->data + nodedata->overhead[0]);
    int fwd = 0; // 0 do not forward, 1 forwar, 2 sink, 3 buffer drop
 
    switch(header->p_type) {
        case BUILD:         
            nodedata->node_status = NODE_ON;
            if( nodedata->seqno < header->p_seqno ) {
                nodedata->seqno = header->p_seqno;
                nodedata->depth = header->p_depth + 1;
                nodedata->from = header->p_src;
                helper++;
            }
            if( nodedata->depth > (header->p_depth + 1) ) {
                nodedata->seqno = header->p_seqno;
                nodedata->depth = header->p_depth + 1;
                nodedata->from = header->p_src;
                helper++;
            }
            if (helper > 0 && nodedata->msg_status == MES_NO ) {
                nodedata->msg_status = MES_BU;
                scheduler_add_callback(get_time() + get_random_time_range(0,entitydata->Delay), c, tx_build, NULL); 
            }
            break;
        case DATA:
            // node is not moving
            if ( header->p_depth > nodedata->depth && nodedata->node_status == NODE_ON ) { 
                if ( nodedata->buffer_pointer < BUFFER - 1  && check_seq(c, header->p_seqno) == 1 ) {
                    if ( nodedata->type == SENSOR ) { // node is a sensor
                        fwd = 1;
                    } else { // node is the sink
                        fwd = 2;
                    }
                } else { // buffer drop
                    fwd = 3;
                }
            }

            if (fwd == 1){
                (nodedata->p[nodedata->buffer_pointer]).p_src     = header->p_src;
                (nodedata->p[nodedata->buffer_pointer]).p_dst     = header->p_dst;
                (nodedata->p[nodedata->buffer_pointer]).p_type    = header->p_type;
                (nodedata->p[nodedata->buffer_pointer]).p_seqno   = header->p_seqno;
                (nodedata->p[nodedata->buffer_pointer]).p_depth   = header->p_depth;
                (nodedata->p[nodedata->buffer_pointer]).p_origin  = header->p_origin;
                (nodedata->p[nodedata->buffer_pointer]).p_status  = header->p_status;
                (nodedata->p[nodedata->buffer_pointer]).p_stamp   = header->p_stamp;
                (nodedata->p[nodedata->buffer_pointer]).p_pos_x   = header->p_pos_x;
                (nodedata->p[nodedata->buffer_pointer]).p_pos_y   = header->p_pos_y;
                nodedata->buffer_pointer ++ ; 
                nodedata->no_packet_recv ++ ;
                add_seq(c, header->p_seqno);
                scheduler_add_callback(get_time() + 
                    get_random_time_range(0,entitydata->Delay), c, tx_forward, NULL);
#ifdef STATS
                printf("[ENERGY] %lli (%i) %lli %i %i me:%i - %i\n", 
                    get_time(), header->p_origin, 
                    get_time()- header->p_stamp, 
                    header->p_seqno,header->p_src,c->node,my_energy(c,0));
#endif
            }

            if (fwd == 2) {
                nodedata->no_packet_recv ++ ;
#ifdef STATS
                printf("%lli (%i) %lli %i %i\n", 
                    get_time(), header->p_origin, 
                    get_time()- header->p_stamp, 
                    header->p_seqno,header->p_src);
#endif
            }

            if (fwd == 3) {
            nodedata->no_packet_drop ++ ;
            }

            break;
        default : 
            break;       
    }
    packet_dealloc(packet);
    return;
}


/* ************************************************** */
/* ************************************************** */
application_methods_t methods = {rx};
