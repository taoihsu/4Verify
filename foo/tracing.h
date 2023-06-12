// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TRACING_H_
#define __TRACING_H_
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// PRQA S 1070 EOF
// PRQA S 1020 EOF
/* Tracing Macro definitions */
#if defined (DEBUG) && defined (TRACING)
#ifdef APP_CTRL
#include "tracer/tracerDB.h"
#define TRACE_0( hTracer, fmt ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt )
#define TRACE_1( hTracer, fmt, p1 ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt, p1 )
#define TRACE_2( hTracer, fmt, p1, p2 ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt, p1, p2 )
#define TRACE_3( hTracer, fmt, p1, p2, p3 ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt, p1, p2, p3 )
#define TRACE_4( hTracer, fmt, p1, p2, p3, p4 ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt, p1, p2, p3, p4 )
#define TRACE_5( hTracer, fmt, p1, p2, p3, p4, p5 ) \
    AddTraceMessage( hTracer, __FILE__, __LINE__, fmt, p1, p2, p3, p4, p5 )
#else
#include <cstdio>
#include <cstring>
#define TRACE_0( hTracer, fmt ) \
    printf("%s: ", __func__); \
    printf(fmt); \
    printf("\n")
#define TRACE_1( hTracer, fmt, p1 ) \
    printf("%s: ", __func__); \
    printf(fmt, p1); \
    printf("\n")
#define TRACE_2( hTracer, fmt, p1, p2 ) \
    printf("%s: ", __func__); \
    printf(fmt, p1, p2); \
    printf("\n")
#define TRACE_3( hTracer, fmt, p1, p2, p3 ) \
    printf("%s: ", __func__); \
    printf(fmt, p1, p2, p3); \
    printf("\n")
#define TRACE_4( hTracer, fmt, p1, p2, p3, p4 ) \
    printf("%s: ", __func__); \
    printf(fmt, p1, p2, p3, p4); \
    printf("\n")
#define TRACE_5( hTracer, fmt, p1, p2, p3, p4, p5 ) \
    printf("%s: ", __func__); \
    printf(fmt, p1, p2, p3, p4, p5); \
    printf("\n")
#endif
#else
#define TRACE_0( hTracer, fmt )                        
#define TRACE_1( hTracer, fmt, p1 )               
#define TRACE_2( hTracer, fmt, p1, p2 )               
#define TRACE_3( hTracer, fmt, p1, p2, p3 )      
#define TRACE_4( hTracer, fmt, p1, p2, p3, p4 )     
#define TRACE_5( hTracer, fmt, p1, p2, p3, p4, p5 )
#endif

// ----------------------------------------------------------------------------
/* Tracing Timers Macro definitions */
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
#include "tracer/timer.h"
#define CREATE_TIMER( timer ) \
    Timer timer
#define START_TIMER( timer ) \
    timer.Start()
#define STOP_TIMER( timer ) \
    timer.Stop()
#define RESET_TIMER( timer ) \
    timer.Reset()
#define GET_ELAPSED_TIME( timer ) \
    timer.GetElapsedTime()
#define GET_LAP( timer ) \
    timer.GetLap()
#else
#define CREATE_TIMER( timer ) 
#define START_TIMER( timer ) 
#define STOP_TIMER( timer ) 
#define RESET_TIMER( timer ) 
#define GET_ELAPSED_TIME( timer ) 0
#define GET_LAP( timer ) 
#endif

// ----------------------------------------------------------------------------
#endif

