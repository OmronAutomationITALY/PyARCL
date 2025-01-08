#%% Imports

import threading as td
import time
import random

#%% Functions Definiton

def program(main, repeatTime = 0, arguments = (), stop_var = False):
    """Function that is used to ease the multithreading process.

    Function that allows easy handling of multithreading requests for multiple functions.
    This could be used to easily send missions in a parallel way or multithreads that
    perform checks to the Robots state and / or store queue information.

    Args:
        main (:py:func:`function`): Function preferably with no arguments that will be used to start the
            multithreading process. If a repeat time is not specified, this function will need to 
            contain the main event loop or the sequence of actions will end after the function completition.

        repeatTime (:py:class:`int`): If this positive value is specified, then the main function will be
            instanced over and over with a time interval equal to the number of seconds specified.
            This can be used to circumvent the event loop in case where the takt time is not fixed but the
            events still need to start occurring at a specified pace.
        
        arguments (:py:class:`tuple`): Tuple containing the arguments of the main function in case it wasn't
            a no-argument function. To use with care.
    """

    if repeatTime <= 0:
        repeat = False
    else:
        repeat = True

    if repeat == True:
        def missionLauncher():
            while not stop_var.is_set():
                td.Thread(target = main, args = arguments).start()
                time.sleep(repeatTime)

        td.Thread(target=missionLauncher).start()
    else:
        td.Thread(target = main, args = arguments).start()


def wait(secs):
    """Function that is used to make the system wait for a specified number of seconds.
    Wraps around time.sleep
    """
    time.sleep(secs)

def waitForJob(job, interval = 1, stop_var = False):
    """Function that is used to make the system wait until the specified :py:class:`~AMRBase.Job` object is completed.
    """
    while not stop_var.is_set():
        try:
            try:
                goFlag = job.iscomplete
            except:
                raise Exception('Object handed is not a Job class')
            
            if goFlag:
                break
            else:
                wait(interval)
        except:
            wait(interval)

def waitForInput(robot, inputName, interval = 1, stop_var = False):
    """Function that is used to make the system wait until the Robot's :py:meth:`~AMRBase.Robot.inputs` method says that the specified input is on.
    """
    while not stop_var.is_set():
        try:
            goFlag = robot.inputs[inputName]
        except:
            raise Exception('Object handed is not a Robot class or input name does not exists')
        
        if goFlag:
            break
        else:
            wait(interval)

def waitForJobState(job, desiredState, jobSegmentNumber = -1, interval = 1, stop_var = False):
    """Function that is used to make the system wait until the specified :py:class:`~AMRBase.Job`'s :py:class:`~AMRBase.JobSegment` object reaches the desired state.
    """
    while not stop_var.is_set():
        try:
            state = job.jobSegments[job.idList[jobSegmentNumber]].state
            if state == desiredState:
                break
            else:
                wait(interval)
        except:
            wait(interval)

def randomNumber(n1, n2):
    """Function that is used to get a random number between two extremes.
    Wraps around random.randomint
    """
    return random.randint(n1, n2)

#%%