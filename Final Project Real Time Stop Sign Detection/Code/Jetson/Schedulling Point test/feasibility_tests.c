/*
 * This code is used for demonstrating feasibility tests for real-time operating systems.
 * Author: Kiran Jojare, Ayswariya Kannan
 * Course: ECEN 5623 Real-Time Operating Systems
 * Reference: Code provided by Professor Sam Siewert
 * University: University of Colorado Boulder
 *
 * The code specifically applies the completion time test, scheduling point test, and
 * rate monotonic least upper bound test for example 2 from a series of examples.
 * These tests help in verifying the feasibility of scheduling real-time tasks under
 * fixed priority scheduling on a single core.
 */

#include <math.h>
#include <stdio.h>

#define TRUE 1
#define FALSE 0
#define U32_T unsigned int

// U = 0.9967
U32_T ex2_period[] = {2, 5, 7, 13};
U32_T ex2_wcet[] = {1, 1, 1, 2};

int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);
int scheduling_point_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);
int rate_monotonic_least_upper_bound(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);
int edf_feasibility(U32_T numServices, U32_T period[], U32_T wcet[]);
int llf_feasibility(U32_T numServices, U32_T period[], U32_T wcet[]);

int main(void)
{ 
    U32_T numServices = 4;
    printf("************************************************\n");
    printf("************************************************\n");
    printf("******** Completion Test Feasibility Example\n");
    printf("************************************************\n");
    printf("************************************************\n\n");

    printf("Ex-2 U=%4.2f%% (C1=1, C2=1, C3=1, C4=2; T1=2, T2=5, T3=7, T4=13; T=D): ",
           ((1.0/2.0)*100.0 + (1.0/5.0)*100.0 + (1.0/7.0)*100.0 + (2.0/13.0)*100.0));

    if(completion_time_feasibility(numServices, ex2_period, ex2_wcet, ex2_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    if(rate_monotonic_least_upper_bound(numServices, ex2_period, ex2_wcet, ex2_period) == TRUE)
        printf("RM LUB FEASIBLE\n");
    else
        printf("RM LUB INFEASIBLE\n");

    if(edf_feasibility(numServices, ex2_period, ex2_wcet) == TRUE)
        printf("EDF FEASIBLE\n");
    else
        printf("EDF INFEASIBLE\n");

    if(llf_feasibility(numServices, ex2_period, ex2_wcet) == TRUE)
        printf("LLF FEASIBLE\n");
    else
        printf("LLF INFEASIBLE\n");

    printf("************************************************\n");
    printf("************************************************\n");
    printf("******** Scheduling Point Feasibility Example\n");
    printf("************************************************\n");
    printf("************************************************\n\n");

    printf("Ex-2 U=%4.2f%% (C1=1, C2=1, C3=1, C4=2; T1=2, T2=5, T3=7, T4=13; T=D): ",
           ((1.0/2.0)*100.0 + (1.0/5.0)*100.0 + (1.0/7.0)*100.0 + (2.0/13.0)*100.0));

    if(scheduling_point_feasibility(numServices, ex2_period, ex2_wcet, ex2_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    return 0;
}

int rate_monotonic_least_upper_bound(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[])
{
    double utility_sum = 0.0, lub;
    printf("for %d, utility_sum = %lf\n", numServices, utility_sum);
    for(int idx = 0; idx < numServices; idx++)
    {
        utility_sum += (double)wcet[idx] / period[idx];
        printf("for %d, wcet=%lf, period=%lf, utility_sum = %lf\n", idx, (double)wcet[idx], (double)period[idx], utility_sum);
    }
    lub = numServices * (pow(2.0, 1.0 / numServices) - 1.0);
    printf("utility_sum = %lf\n", utility_sum);
    printf("LUB = %lf\n", lub);
    return utility_sum <= lub ? TRUE : FALSE;
}

int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[])
{
    for (int i = 0; i < numServices; i++)
    {
        U32_T an = 0, anext;
        for (int j = 0; j <= i; j++)
        {
            an += wcet[j];
        }
        do
        {
            anext = wcet[i];
            for (int j = 0; j < i; j++)
            {
                anext += ceil((double)an / period[j]) * wcet[j];
            }
            if (anext == an)
                break;
            an = anext;
        } while (1);

        if (an > deadline[i])
            return FALSE;
    }
    return TRUE;
}

int scheduling_point_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[])
{
    int rc = TRUE;
    for (int i = 0; i < numServices; i++)
    {
        int status = 0;
        for (int k = 0; k <= i; k++)
        {
            for (int l = 1; l <= floor((double)period[i] / period[k]); l++)
            {
                U32_T temp = 0;
                for (int j = 0; j <= i; j++)
                {
                    temp += wcet[j] * ceil((double)(l * period[k]) / period[j]);
                }
                if (temp <= (l * period[k]))
                {
                    status = 1;
                    break;
                }
            }
            if (status) break;
        }
        if (!status) rc = FALSE;
    }
    return rc;
}

int edf_feasibility(U32_T numServices, U32_T period[], U32_T wcet[]) {
    double totalUtilization = 0.0;
    for (int i = 0; i < numServices; i++) {
        totalUtilization += (double)wcet[i] / period[i];
    }
    printf("EDF Total Utilization: %f\n", totalUtilization);
    return totalUtilization <= 1.0 ? TRUE : FALSE;
}

int llf_feasibility(U32_T numServices, U32_T period[], U32_T wcet[]) {
    double totalUtilization = 0.0;
    for (int i = 0; i < numServices; i++) {
        totalUtilization += (double)wcet[i] / period[i];
    }
    printf("LLF Total Utilization: %f\n", totalUtilization);
    return totalUtilization <= 1.0 ? TRUE : FALSE;
}

