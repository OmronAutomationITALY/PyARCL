##############################################################################
# PyARCL is a personal project of Marco Zangrandi, from OMRON Italy (OEE-IT)
# Contact me at marco.zangrandi@omron.com
# Jan 2022 - Dec 2022
##############################################################################

#%% Imports

import telnetlib
from datetime import datetime
import threading
import logging
import re
import inspect
from functools import wraps
import os

#%% Logger Setup

def setupLogger(name, log_file, logLevel = 'info'):
    """Function that generates a custom logger with custom filename in the logs folder.
    """
    leveldict = {
        'ultradebug': logging.DEBUG,
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL
    }
    if logLevel not in leveldict:
        raise Exception('"logLevel" parameter must be either debug, info, warning, error or critical.')
    level = leveldict[logLevel]

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    if not os.path.exists('logs'):
        os.mkdir('logs')
    handler = logging.FileHandler(f'logs\\{log_file}', mode = 'w')        
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger

generalLog = setupLogger('GeneralLog', 'GeneralLog.log')

#%% General Purpose Functions

def sanitizeLogMessage(string):
    '''Sanitizes strings that need to be put inside the log.

    Function that takes a string that might be on multiple lines and makes it one 
    line only.
    The function takes the string and checks if it ends with "\\r\\n"; if it does 
    returns said string without "\\r\\n", returns the original string otherwise.
    '''
    returnStrokeIndices = [m.start() for m in re.finditer('\r\n', string)]
    returnStrokeLen = 2
    if not returnStrokeIndices:
         returnStrokeIndices = [m.start() for m in re.finditer('\n', string)]
         returnStrokeLen = 1
        #  print(returnStrokeIndices)
    
    subStrings = []
    if returnStrokeIndices:
        first_substring = f'[1]: {string[:returnStrokeIndices[0]]};'
        subStrings.append(first_substring)
        
        if  not len(returnStrokeIndices) == 1:
            for i in range(len(returnStrokeIndices[:-1])):
                ith_string = f'[{i+2}]: {string[returnStrokeIndices[i]+2:returnStrokeIndices[i+1]]};'
                subStrings.append(ith_string)
        else:
            i = 1
        
        # print(repr(string[returnStrokeIndices[-1]+2:]))
        if string[returnStrokeIndices[-1]+2:]:
            last_substring = f'[{i+1}]: {string[returnStrokeIndices[-1]+2:]};'
            subStrings.append(last_substring)

        if len(subStrings) == 1:
            return string[:-returnStrokeLen]

        return ' '.join(subStrings)
    else:
        return string    
    
# Give 'sanitizeLogMessage' function a more usable alias
slm = sanitizeLogMessage

def sendCommand(TL_obj, command, logger = generalLog):
    '''Sends an ARCL command to a mobile robot or fleet manager.

    Args:
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        command (:py:class:`str`): The command string to send to the server.

        logger (:py:class:`~logging.Logger`): The logger to log to.

    Returns:
        Nothing
    '''

    # If a 'Robot' or 'FleetManager' object is passed to the function, their TL_obj
    # and logger are passed, otherwise use the standard one.
    if type(TL_obj) is not telnetlib.Telnet:
        logger = TL_obj.logger
        TL_obj = TL_obj.TL_obj

    command_text = (command + '\n').encode('ascii')
    # Logging the sent message
    logger.debug(f'Sent "{slm(command)}" to {TL_obj.host}')
    TL_obj.write(command_text)

def findLine(TL_obj, line, timeout = 10, checkExceptions = True, logger = generalLog):
    '''Searches for a line inside the Telnet server.

    Function that reads through the lines of a Telnet server until specified line 
    is found inside the incoming messages; returns said message.

    Args:
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

        line (:py:class:`str`): String to search for in the incoming Telnet messages

        timeout (:py:class:`int`): Number of seconds to run until function is killed and
            an Exception is raised.
        
        checkExceptions (:py:class:`bool`): Boolean value that checks for exceptions in the
            ARCL command lines and raises the appropriated exception in case it finds one.

        logger (:py:class:`~logging.Logger`): The logger to log to.

    Returns:
        (:py:class:`str`): String containing the message containing the searched line.
    '''

    # If a 'Robot' or 'FleetManager' object is passed to the function, their TL_obj
    # and logger are passed, otherwise use the standard one.
    if TL_obj is not telnetlib.Telnet:
        logger = TL_obj.logger
        TL_obj = TL_obj.TL_obj
    
    startTime = datetime.now()

    # Logging the findLine event
    logger.debug(f'Started reading lines from {TL_obj.host}, searching for "{slm(line)}"')
    i = 0
    while True:
        sel_line = readLine(TL_obj, logger)

        if checkExceptions:
            if 'Unknown command' in sel_line:
                commandName = sel_line[16:-2]
                logger.error(f'findLine function read "{commandName}" unknown command exception')
                raise Exception(f'Read "Unknown command {commandName}"')
            elif 'CommandError' in sel_line:
                commandName = sel_line[14:-2]
                err_desc_line = readLine(TL_obj, logger)
                if 'CommandErrorDescription' in err_desc_line:
                    err_desc = err_desc_line[25:-2]
                    print(err_desc_line)
                    logger.error(f'findLine function read error related to "{commandName}", error description says "{err_desc}"')
                    raise Exception(f'Command "{commandName}" returns error "{err_desc}"')
                else:
                    logger.error(f'findLine function read error related to "{commandName}"')
                    raise Exception(f'Command "{commandName}" returns error')

        if line in sel_line:
            # Logging the line find
            logger.debug('Line found')
            break
        elif (datetime.now() - startTime).total_seconds() > timeout:
            # Logging the findLine exception
            logger.error(f'findLine function timeout reached')
            raise Exception('String not found')
        i += 1
        
    return sel_line

def readLine(TL_obj, logger = generalLog, timeout = 5):
    '''Reads the current line of a Telnet server and returns said line.

    Args:
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

        logger (:py:class:`~logging.Logger`): The logger to log to.

    Returns:
        (:py:class:`str`): Line read.
    '''
    
    line = TL_obj.read_until(b'\n', timeout = timeout).decode('ascii')
    # Logging the read line
    logger.debug(f'Read "{slm(line)}" from {TL_obj.host}')
    return line

def trimmer(string, start = ':', length = 2, end = False):
    '''Function used to trim a string, allows for multiple input types.
    '''

    # Multi input type in 'start'; since most feedbacks are of the kind 'Feedback: Data'
    # then the default value of 'start' is ':'. 'start' can also be defined as an int
    # and in that case it is used as the starting point of the trimming.
    if isinstance(start, str):
        ind_i = string.index(start)
    else:
        ind_i = start

    # Multi input type in 'end'; since most feedbacks are of the kind 'Feedback: Data'
    # then the default value of 'end' is '\r\n' as the character for the new line.
    if end != False:
        if isinstance(end, str):
            ind_o = string.index(end)
        else:
            ind_o = end
    else:
        ind_o = string.index('\r\n')

    # 'length' is the character length to remove to the start of the trimmed
    # string. Since the typical feedback is of the likes of 'Feedback: Data'
    # then 'length' default value is 2. This way the ': Data' string gets
    # automatically trimmed to 'Data'.
    return string[ind_i+length:ind_o]

def find_nth(haystack, needle, n):
    '''Finds the nth element index inside a container.

    Fuction used to find the nth occurrance of an element inside a container
    and returns its index. Taken by BitFlip response on StackOverflow 
    (https://stackoverflow.com/questions/73902551).
    '''
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start+len(needle))
        n -= 1
    return start

def onoffToBool(string):
    '''Translates 'on' or 'off' to their boolean counterpart.

    Function that takes a string in input and returns a boolean based on whether 
    the original string was "on" or "off".
    '''
    if string == 'on':
        return True
    elif string == 'off':
        return False
    elif string == 'broken':
        return 'Broken'
    else:
        raise Exception('String neither on or off')

def startServer(HOST_IP, password, HOST_PORT = 7171, logger = generalLog):
    '''
    Function that creates a telnet object and sends the needed password through 
    to open it. Standard function to use when opening a socket connection with 
    '''

    # Logging the starting of the connection
    logger.info(f'Attempting to start Telnet server on {HOST_IP}:{HOST_PORT}')
    TL_obj = telnetlib.Telnet(HOST_IP, HOST_PORT)
    TL_obj.write((f'{password}\n').encode('ascii'))
    # Logging the suffessful connection of the Telnet server
    logger.info(f'Telnet server on {HOST_IP}:{HOST_PORT} started correctly')
    return TL_obj

#%% Multithreading Support Decorator

def TL_request(request):
    '''Decorator to handle mutex operations in multithreading access to Telnet server.

    This function is meant to be used as a decorator for the MasterAMR class 
    and subclasses to allow for ordered multi-threading access to the
    Telnet textserver.
    '''
    
    # Needed to correct docstring import...
    @wraps(request)
    # This monstruosity of nested functions is needed to define a decorator
    # that has access to the 'self' class object property, since a decorator
    # can only be defined as a function with single argument that only
    # returns a function.
    # 'TL_requestAdj' collects the 'self' method and the various arguments of
    # 'request' as *args, **kw. It's used to then create a zero-argument function
    # that is still able to call the original 'request' correctly.
    def TL_requestAdj(self, *args, **kw):
        
        # 'adjRequest' is a function with zero arguments that represents the 
        # original 'request' call with its arguments intact. This is needed to 
        # put it through a multithreading pool.
        def adjRequest():
            out = request(self, *args, **kw)
            return out

        # If 'request' is called by a thread, save the original thread ID
        originalThread = threading.get_ident()
        # create the thread that will generate the 'request' instance, then
        # put it in the queue
        t1 = TelnetRequestThread(target = adjRequest, TL_requests = self.TL_requests, origin = originalThread, logger = self.logger)
        
        self.TL_requests.lock.acquire()
        self.TL_requests.append(t1)
        self.TL_requests.lock.release()

        # Start the thread and wait for it to complete correctly
        t1.start()
        t1.join()

        # Remove the thread in the thread queue
        self.TL_requests.lock.acquire()
        self.TL_requests.remove(t1)
        self.TL_requests.lock.release()

        # Return to base level the exceptions that were on threading level
        if t1.inError:
            raise(t1.errorMessage)
        
        return t1.returnValue
    
    # Since the only returned value from 'TL_request' is a function it is
    # allowed to be used as a decorator
    return TL_requestAdj

class TelnetRequestThread(threading.Thread):
    '''Class that describes a thread with custom mutex functionality.

    This class is meant to be used by the TL_request function.
    Inherits most properties from the threading.Thread class.
    '''
    
    # init method first calls needed actions from the parent class, then saves
    # a few variables to be used in the thread queue management
    def __init__(self, target, TL_requests, origin, logger):
        threading.Thread.__init__(self)

        self.target = target
        self.returnValue = None
        self.TL_requests = TL_requests
        self.origin = origin
        self.logger = logger

        # Exception-handling parameters
        self.errorMessage = None
        self.inError = False

    # The run function runs when the thread is first started 
    def run(self):
        # Logging the current thread asking for Telnet access
        self.logger.debug(f'thread {threading.get_ident()} requests access to Telnet')
        # 'TL_request' thread elements are inspected one by one and joined by
        # the current thread. When the current thread sees itself in the list
        # it's its turn and it is allowed to access the Telnet textserver.
        # Only exception to this rule is if the current thread finds his parent
        # waiting for completition, in which case he goes first since the parent
        # is waiting for him to finish before moving on. This is used to avoid
        # otherwise inevitable deadlocks. Please note this only allows for 1-level
        # thread nesting.
        # 'TL_request' is copied at the thread start to allow indexes to make 
        # sense since the original list is modified all the time by popping
        # and adding elements.
        
        self.TL_requests.lock.acquire()
        localRequests = self.TL_requests.copy()
        self.TL_requests.lock.release()

        threadIdList = [thread.ident for thread in localRequests]

        for thread in localRequests:
            if thread.ident == threading.get_ident():
                break
            elif self.origin in threadIdList:
                originIndex = threadIdList.index(self.origin)
                if thread.ident == self.origin:
                    break
                if thread.ident == localRequests[originIndex].origin:
                    break
            else:
                # if the thread to wait for finishes and gets deleted before
                # the current thread is able to join it correctly, the
                # 'join' method will raise an exception. In this case,
                # we can consider the thread to be finished and move along.
                try:
                    # Logging the thread waiting for another thread to finish
                    self.logger.debug(f'thread {threading.get_ident()} waiting for thread {thread.ident}')
                    thread.join()
                except:
                    continue
        
        # Logging the Telnet server access
        self.logger.debug(f'thread {threading.get_ident()} granted access to Telnet')  

        # Running the actual function
        try:
            self.returnValue = self.target()
        except Exception as errorMessage:
            self.errorMessage = errorMessage
            self.inError = True
        
        # Logging the Telnet server leave
        self.logger.debug(f'thread {threading.get_ident()} exiting Telnet')

class ThreadSafeList(list):
    def __init__(self, *args, **kwargs):
        list.__init__(self, *args, **kwargs)
        self.lock = threading.Lock()


#%% Subclasses Definition

class JobSegment:
    '''Class describing a job segment.

    This class describes a job segment, it is meant to be used to make other 
    functions return JobSegment objects, not to be used by the user directly.

    Attributes:        
        id (:py:class:`str`): The job segment unique id.

        jobid (:py:class:`str`): The job id this job segment belongs to.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

        islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
            Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
            to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
            only way to do so is by using the queueDropoff ARCL command.

        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''

    def __init__(self, TL_obj, id, jobid, mapgoalList = [], islocal=False, TL_requests = [], logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            id (:py:class:`str`): The job segment unique id.

            jobid (:py:class:`str`): The job id this job segment belongs to.

            mapgoalList (:py:class:`list` of :py:class:`str`): List containing the names of all the
                goals present in map.

            islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
                Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
                to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
                only way to do so is by using the queueDropoff ARCL command.

            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
            
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.id = id
        self.jobid = jobid
        self.__goalList = mapgoalList
        self.islocal = islocal
        self.TL_requests = TL_requests
        self.logger = logger
    
    @property
    def type(self):
        '''string: Type of job segment. Can be either pickup or dropoff.
        '''

        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" type')
        if 'PICKUP' in self.id:
            # Logging result
            self.logger.info(f'Returned "Pickup" for job segment "{self.id}" type')
            return 'Pickup'
        elif 'DROPOFF' in self.id:
            # Logging result
            self.logger.info(f'Returned "Dropoff" for job segment "{self.id}" type')
            return 'Dropoff'
        else:
            # Logging the exception
            self.logger.error(f'Job Segment "{self.id}" type neither Pickup nor Dropoff')
            raise Exception('job segment id neither dropoff or pickup. This should not be happening; please report.')

    # The data items of a 'JobSegment' object are priority, state, substate, 
    # goal, assignedRobot, queuedTime, finishedTime and failNumber. All of 
    # those are got by the same command over ARCL and are therefore parsed 
    # by the same function. Then each of those items is assigned a property.
    @TL_request
    def _get_info(self):
        # The 'info' class is used only to generate the object that will
        # collect all the different information pieces, so that it might be
        # used easily for property declaration.
        class info:
            def __init__(self, priority, state, substate, goal, assignedRobot, queuedTime, finishedTime, failNumber):
                self.priority = priority
                self.state = state
                self.substate = substate
                self.goal = goal
                self.assignedRobot = assignedRobot
                self.queuedTime = queuedTime
                self.finishedTime = finishedTime
                self.failNumber = failNumber
        
        # queuequery is the command that queries the jobSegment info based
        # on id. Response is of the form of "QueueQuery: Data".
        if self.islocal:
            sendCommand(self, f'queuequerylocal id {self.id}')
        else:
            sendCommand(self, f'queuequery id {self.id}')


        line = findLine(self, f'QueueQuery: {self.id}')
        
        priority = int(trimmer(line, find_nth(line, ' ', 3), 1, find_nth(line, ' ', 4)))
        state = trimmer(line, find_nth(line, ' ', 4), 1, find_nth(line, ' ', 5))
        substate = trimmer(line, find_nth(line, ' ', 5), 1, find_nth(line, ' ', 6))
        goal = trimmer(line, find_nth(line, ' ', 7), 2, find_nth(line, ' ', 8)-1)
        assignedRobot = trimmer(line, find_nth(line, ' ', 8), 2, find_nth(line, ' ', 9)-1)

        # 'queuedTime' and 'finishedTime' can be "None" as it might be that
        # the job is not assigned and/or not finished. This snippet of code
        # transforms the "None" string in the 'None' Python object.
        if trimmer(line, find_nth(line, ' ', 9), 1, find_nth(line, ' ', 10)) == 'None':
            queuedTime = None
        else:
            queued_time_str = trimmer(line, find_nth(line, ' ', 9), 1)[:19]
            queuedTime = datetime.strptime(queued_time_str, '%m/%d/%Y %H:%M:%S')

        if trimmer(line, find_nth(line, ' ', 11), 1, find_nth(line, ' ', 12)) == 'None':
            finishedTime = None
        else:
            finished_time_str = trimmer(line, find_nth(line, ' ', 11), 1)[:19]
            finishedTime = datetime.strptime(finished_time_str, '%m/%d/%Y %H:%M:%S')

        failNumber = trimmer(line, line.index('""'), 3)

        # Set the pointer to the 'EndQueueQuery' line
        findLine(self, 'EndQueueQuery')

        return info(priority, state, substate, goal, assignedRobot, queuedTime, finishedTime, failNumber)
    
    # Defining get and set functions for property declaration
    def _get_priority(self):
        '''int: The priority value of the job segment.

        Can be set: Setting the priority to another value will result as such
        if the job segment is not completed yet.
        '''
        # Logging the jobSegment priority request
        self.logger.info(f'Requested job segment "{self.id}" priority')
        priority = self._get_info().priority
        # Logging the returned value
        self.logger.debug(f'Returned "{priority}" for job segment "{self.id}"')
        return priority

    def _get_state(self):
        '''string: The job state as described by ARAM.
        '''
        # Logging the jobSegment state request
        self.logger.info(f'Requested job segment "{self.id}" state')
        state = self._get_info().state
        # Logging the returned value
        self.logger.debug(f'Returned "{state}" for job segment "{self.id}"')
        return state

    def _get_substate(self):
        '''string: The job substate as described by ARAM.
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" substate')
        substate = self._get_info().substate
        # Logging the returned value
        self.logger.debug(f'Returned "{substate}" for job segment "{self.id}"')
        return substate

    def _get_goal(self):
        '''string: The map goal the job segment is associated with.

        Can be set: Setting this goal to another in map will result as such
        if the job segment is not completed yet. Will raise an exception if
        the new goal does not exist in map. 
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" goal')
        goal = self._get_info().goal
        # Logging the returned value
        self.logger.debug(f'Returned "{goal}" for job segment "{self.id}"')
        return goal

    def _get_assignedRobot(self):
        '''string: The unique id of the AMR that is assigned to the job segment.
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" assignedRobot')
        assignedRobot = self._get_info().assignedRobot
        # Logging the returned value
        self.logger.debug(f'Returned "{assignedRobot}" for job segment "{self.id}"')
        return assignedRobot

    def _get_queuedTime(self):
        '''Datetime object: Time at which the job segment was successfully accepted 
        by the AMR or FM queuing manager.
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" queuedTime')
        queuedTime = self._get_info().queuedTime
        # Logging the returned value
        self.logger.debug(f'Returned "{queuedTime}" for job segment "{self.id}"')
        return queuedTime

    def _get_finishedTime(self):
        '''Datetime object: Time at which the job segment was completed.
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" finishedTime')
        finishedTime = self._get_info().finishedTime
        # Logging the returned value
        self.logger.debug(f'Returned "{finishedTime}" for job segment "{self.id}"')
        return finishedTime

    def _get_failNumber(self):
        '''int: Number of times the job segment has failed before.
        '''
        # Logging the jobSegment type request
        self.logger.info(f'Requested job segment "{self.id}" failNumber')
        failNumber = self._get_info().failNumber
        # Logging the returned value
        self.logger.debug(f'Returned "{failNumber}" for job segment "{self.id}"')
        return failNumber

    # Goal and priority can be changed for a goal segment by using the
    # "queuemodify" ARCL command.
    @TL_request
    def _set_goal(self, newGoal):
        # Logging the jobSegment goal set attempt
        self.logger.info(f'Attempting to set job segment "{self.id}" goal as "{newGoal}"')
        if self.__goalList:
            if newGoal not in self.__goalList:
                # Logging the exception
                self.logger.error(f'Attempted to modify "{self.id}" goal to "{newGoal}", which is not present in map')
                raise Exception('Cannot modify job segment goal: Goal name not in map') 
        
        if self.islocal:
            sendCommand(self, f'queuemodifylocal {self.id} goal {newGoal}')
        else:
            sendCommand(self, f'queuemodify {self.id} goal {newGoal}')
        
        findLine(self, 'AfterModify')

    @TL_request
    def _set_priority(self, newPriority):
        # Logging the jobSegment priority set attempt
        self.logger.info(f'Attempting to set job segment "{self.id}" priority as "{newPriority}"')
        
        if self.islocal:
            sendCommand(self, f'queuemodifylocal {self.id} priority {str(newPriority)}')
        else:
            sendCommand(self, f'queuemodify {self.id} priority {str(newPriority)}')
        
        findLine(self, 'AfterModify')

    # Dummy set function that does nothing. For property declaration.
    def _set_any(self, value):
        pass
    
    # Property declaration
    priority = property(_get_priority, _set_priority)
    state = property(_get_state, _set_any)
    substate = property(_get_substate, _set_any)
    goal = property(_get_goal, _set_goal)
    assignedRobot = property(_get_assignedRobot, _set_any)
    queuedTime = property(_get_queuedTime, _set_any)
    finishedTime = property(_get_finishedTime, _set_any)
    failNumber = property(_get_failNumber, _set_any)

    def __repr__(self):
        info = self._get_info()

        description = f'''
        JobSegment: {self.id}
            JobID:          {self.jobid}
            JobType:        {self.type}

            Priority:       {info.priority}
            State:          {info.state}
            Substate:       {info.substate}
            Goal:           {info.goal}
            AssignedRobot:  {info.assignedRobot}
            QueuedTime:     {info.queuedTime}
            FinishedTime:   {info.finishedTime}
            FailNumber:     {info.failNumber}
        '''
        return inspect.cleandoc(description)

class Job:
    '''Class describing a job.
    
    This class describes a job, mostly contains basic informations and a dictionary
    containing its segments. It is meant to be used by other functions to return a
    Job object, it is not meant to be used by the user.
    The :py:class:`~pyarcl.Job` object can be also used as it was a dictionary. When this happens, it
    defaults at pointing its :py:attr:`~pyarcl.Job.jobSegments` attribute.
    E.g. Job.jobSegments[jobSegmentName] and Job[jobSegmentName] are essentially 
    the same thing.

    Attributes:
        jobid (:py:class:`str`): The job unique id.

        idList (:py:class:`list` of :py:class:`str`): List containing the names of all the job segments
            that the job possesses.
        
        jobSegments (:py:class:`~AMRBase.JobSegmentDict`): Container that can be used to access
            all the job segments this job possesses as JobSegment objects. Acts
            like a dictionary returning JobSegment objects.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
            Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
            to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
            only way to do so is by using the queueDropoff ARCL command.

        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, idList, jobid, JobSegmentDictObject, islocal, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            idList (:py:class:`list` of :py:class:`str`): List containing the names of all the job segments
                that the job possesses.

            jobid (:py:class:`str`): The job unique id.
            
            jobSegmentDictObject (:py:class:`~AMRBase.JobSegmentDict`): Container that can be used
                to access all the job segments this job possesses as JobSegment 
                objects. Acts like a dictionary returning JobSegment objects.

            islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
                Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
                to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
                only way to do so is by using the queueDropoff ARCL command.

            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
            
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.idList = idList
        self.jobid = jobid
        self.jobSegments = JobSegmentDictObject
        self.islocal = islocal
        self.TL_requests = TL_requests
        self.logger = logger
    
    @TL_request
    def cancel(self):
        '''No arguments method that cancels the job.
        '''
        # Logging the Job cancel attempt
        self.logger.info(f'Attempting to cancel job "{self.jobid}"')

        if self.islocal:
            sendCommand(self, f'queuecancellocal jobid {self.jobid}')
        else:
            sendCommand(self, f'queuecancel jobid {self.jobid}')
    
    def __getcompletition(self):
        if self.jobSegments[self.idList[-1]].state == 'Completed':
            return True
        else:
            return False
    
    @property
    def iscomplete(self):
        ''':py:class:`bool`: Checks whether the job is completed or not.

        This is done by accessing the state of the last job segment and checking
        whether that is 'Completed' or not.
        '''
        # Logging the job completition request
        self.logger.info(f'Requested job "{self.jobid}" completition')
        completition = self.__getcompletition()
        # Logging the returned value
        self.logger.debug(f'Returned "{completition}" for job "{self.jobid}"')
        return completition
    
    # Assigned robot is always the same one, choosing one 'JobSegment' object
    # is no different than choosing another.
    @property
    def assignedRobot(self):
        ''':py:class:`str`: Robot that is assigned to this job. This is done by checking the
        robot assigned to the last job segment.
        '''
        # Logging the job completition request
        self.logger.info(f'Requested job "{self.jobid}" assignedRobot')
        assignedRobot = self.jobSegments[self.idList[-1]].assignedRobot
        # Logging the returned value
        self.logger.debug(f'Returned "{assignedRobot}" for job "{self.jobid}"')
        return assignedRobot
    
    # Following methods are intended to make the 'Job' object act like the
    # 'self.jobSegments' object when accessed directly
    def __getitem__(self, jobSegmentName):
        return self.jobSegments[jobSegmentName]

    def __repr__(self):
        description = f'Job: {self.jobid}\r\n'
        description += f'    Completition: {self.iscomplete}\r\n\r\n'

        description += '    Job Segments:\r\n'
        for i, id in enumerate(self.idList):
            description += f'        [{i+1}]: {id}\r\n'
        # Remove last unnecessary '\r\n'
        description = description[:-2]

        return description

    def __contains__(self, jobSegmentName):
        if jobSegmentName in self.idList:
            return True
        else:
            return False
    
    def __iter__(self):
        for jobSegmentName in self.idList:
            yield self.jobSegments[jobSegmentName]

class JobSegmentDict(dict):
    '''Container acting as a dictionary that returns :py:class:`~AMRBase.JobSegment` objects.
    
    Class not meant to be used by the user. It is used to generate the :py:class:`~AMRBase.JobSegment`
    objects that are contained within the :py:class:`~AMRBase.Job` objects.
    This class behaves like a dictionary.

    Attributes:
        jobid (:py:class:`str`): The job unique id.

        idList (:py:class:`list` of :py:class:`str`): List containing the names of all the job segments
            that the job possesses.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

        islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
            Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
            to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
            only way to do so is by using the queueDropoff ARCL command.

        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, idList, jobid, mapgoalList, islocal, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            idList (:py:class:`list` of :py:class:`str`): List containing the names of all the job segments
                that the job possesses.

            jobid (:py:class:`str`): The job unique id.
            
            mapgoalList (:py:class:`list` of :py:class:`str`): List containing the names of all the
                goals present in map.

            islocal (:py:class:`bool`): Boolean that discriminates if the job was sent to an Enterprise
                Manager or standalone Robot with 'Queuing Manager' active (islocal = False) or directly
                to a single Robot connected to an Enterprise Manager (islocal = True). At the moment, the
                only way to do so is by using the queueDropoff ARCL command.

            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
            
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.idList = idList
        self.jobid = jobid
        self.__goalList = mapgoalList
        self.islocal = islocal
        self.TL_requests = TL_requests
        self.logger = logger
    
    def __getitem__(self, jobSegmentId):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to access "{self.jobid}" jobSegment "{jobSegmentId}"')
        if jobSegmentId not in self.idList:
            # Logging the exception
            self.logger.error(f'Requested job segment id "{jobSegmentId}" which is not between "{self.jobid}" segments')
            raise Exception(f'Job segment id "{jobSegmentId}" not between job "{self.jobid}" segments')
        
        return JobSegment(self.TL_obj, jobSegmentId, self.jobid, self.__goalList, self.islocal, self.TL_requests, self.logger)

    def __setitem__(self, key, newValue):
        pass

    def __repr__(self):
        description = 'Job Segments:\r\n'
        for i, id in enumerate(self.idList):
            description += f'    [{i+1}]: {id}\r\n'
        # Remove last unnecessary '\r\n'
        description = description[:-2]

        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, jobSegmentName):
        if jobSegmentName in self.idList:
            return True
        else:
            return False
    
    def __iter__(self):
        for jobSegmentName in self.idList:
            yield self.__getitem__(jobSegmentName)

class JobDict(dict):
    '''Container acting as a dictionary that returns :py:class:`~AMRBase.Job` objects.
    
    Class not meant to be used by the user. It is used to make the queue property
    work for both the :py:class:`~AMRBase.Robot` and :py:class:`~AMRBase.FleetManager` classes.
    This class behaves like a dictionary. Note that thanks to the :py:class:`~AMRBase.JobSegment` class
    allowing to be accessed like a dictionary (defaulting to the :py:attr:`~AMRBase.Job.jobSegments`) object
    the grammar to access a single job segment can be nested.
    E.g. JobDictObject[jobName][jobSegmentName] is essentially identical to
    JobDictObject[jobName].jobSegments[jobSegmentName]

    Attributes:
        queueList (:py:class:`list` of :py:class:`str`): List containing the names of all the jobs
            currently in the queue

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, queueList, mapgoalList, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            queueList (:py:class:`list` of :py:class:`str`): List containing the names of all the job
                currently in the queue

            mapgoalList (:py:class:`list` of :py:class:`str`): List containing the names of all the
                goals present in map.

            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
            
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.queueList = queueList
        self.__goalList = mapgoalList
        self.TL_requests = TL_requests
        self.logger = logger
    
    @TL_request
    def __getitem__(self, jobName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to access "{jobName}" job from the queue')
        if jobName not in self.queueList:
            # Logging the exception
            self.logger.error(f'Job "{jobName}" not in queue')
            raise Exception(f'Job "{jobName}" not (or not anymore) in queue')

        sendCommand(self, f'queuequery jobid {jobName}')

        idList = []
        while True:
            readString = findLine(self, 'QueueQuery')
            if readString == 'EndQueueQuery\r\n':
                break
            id = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
            idList.append(id)

        tempDict = JobSegmentDict(self.TL_obj, idList, jobName, self.__goalList, False, self.TL_requests, self.logger)

        return Job(self.TL_obj, idList, jobName, tempDict, False, self.TL_requests, self.logger)
    
    def __setitem__(self, key, newValue):
            pass

    @TL_request
    def __repr__(self):
        queueList = self.queueList

        fullCommand = ''
        for jobid in queueList:
            fullCommand += f'queuequery jobid {jobid}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'queueList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)
        
        description = 'Jobs:\r\n'

        for i, jobid in enumerate(queueList):
            description += f'    [{i+1}] {jobid}\r\n'
            j = 1
            while True:
                readString = findLine(self, 'QueueQuery')
                if readString == 'EndQueueQuery\r\n':
                    break
                id = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
                description += f'        [{i+1}.{j}] {id}\r\n'
                j += 1
        
        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, jobName):
        if jobName in self.queueList:
            return True
        else:
            return False
        
    def __iter__(self):
        for jobName in self.queueList:
            yield self.__getitem__(jobName)

class Parameter:
    '''Class that describes a datastore parameter.
    
    This class describes a data parameter coming from datastore. It is not meant
    to be used by the user, instead it is meant to be used by the datastore functions.'
    '''
    # Both parameter instances need to be defined as belonging to a section
    def __init__(self, TL_obj, paramName, sectionName, rawValue, logger = generalLog):
        self.sectionName = sectionName
        self.paramName = paramName
        self.rawValue = rawValue
        self.TL_obj = TL_obj
        self.logger = logger
    
    # This function takes the string coming from the Telnet textserver
    # and converts them in either int, float or boolean
    def __adjustValue(self, value):
        def isint(string):
            try:
                val = int(string)
                return True
            except ValueError:
                return False
        
        def isfloat(string):
            try:
                val = float(string)
                return True
            except ValueError:
                return False

        if value == 'true':
            return True
        elif value == 'false':
            return False
        elif value == "None":
            return None
        elif isint(value):
            return int(value)
        elif isfloat(value):
            return float(value)
        else:
            return value

    def _get_value(self):
        # Logging the getvalue attempt
        self.logger.debug(f'Attempting to get "{self.paramName}" value')
        value = self.__adjustValue(self.rawValue)
        # Logging the returned value
        self.logger.debug(f'Parameter {self.paramName} returns value "{value}"')
        return value

    # The set function is not defined as this class will be dedicated
    # to datastore parameters, who cannot be set. Config parameters,
    # who can be set, will redefine this function and make use of this
    # already exisectionNamesting infrastructure through inheritance.
    def _set_value(self, newValue):
        pass
    
    # I need to use this grammar to define the 'value' property otherwise
    # the 'value' property won't call the right "newly defined" function
    # when it is inherited.
    value = property(fget = lambda self: self._get_value(), fset = lambda self, newValue: self._set_value(newValue))

    def __repr__(self):
        return str(self.value)

class ConfigParameter(Parameter):
    '''Class that describes a configuration parameter.

    This class describes a config. parameter; basically just inherits :py:class:`~AMRBase.Parameter` 
    properties and implements a set function for the value property.
    This class is not meant to be used by the user. Instead it is meant to be used 
    by the config functions.
    '''
    def __init__(self, TL_obj, paramName, sectionName, rawValue, TL_requests, logger = generalLog):
        self.sectionName = sectionName
        self.paramName = paramName
        self.rawValue = rawValue
        self.TL_obj = TL_obj
        self.TL_requests = TL_requests
        self.logger = logger

    @TL_request
    def _set_value(self, newValue):
        # Logging the set value attempt
        self.logger.info(f'Attempting to set {self.paramName} value to {newValue}')
        
        # NB: The following code snippet has been edited out since there is no
        # easy way to access the 'config' property inside the 'ConfigSectionDict' class
        # anymore. This might get reimplemented in the 'config' property directly
        # by asking the robot to access this value first and sending downstream
        # the value to all parameters, even though it may slow down everything.
        
        # if not self.config['ARCL server setup']['ArclConfig']:
        #     # Logging the exception
        #     self.logger.error('"ArclConfig" parameter is set to False. Cannot modify configuration values.')
        #     raise Exception('"ArclConfig" parameter is set to False. Cannot modify configuration values.')

        sendCommand(self, 'configstart')
        sendCommand(self, f'configadd Section {self.sectionName}')
        sendCommand(self, f'configadd {self.paramName} {str(newValue)}')
        sendCommand(self, 'configparse')
        findLine(self, 'Configuration changed')

class InfoDict(dict):
    '''Class that acts like a dictionary of info values.

    This class acts like a dictionary, returning info values when items are accessed.
    Info values are returned as strings. The single info values can also be set, but this only
    works if the info was a custom info created by the 'createInfo' ARCL command.
    This class is not meant to be used by the user, instead it is meant to be
    accessed by the :py:attr:`~AMRBase.MasterAMR.info` property of the :py:class:`~AMRBase.MasterAMR` objects.

    Attributes:
        infoList (:py:class:`list` of :py:class:`str`): List containing all the names of the info.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
            
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, infoList, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            infoList (:py:class:`list` of :py:class:`str`): List containing all the names of the info.
        
            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.

            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.infoList = infoList
        self.TL_requests = TL_requests
        self.logger = logger

    def __adjustValue(self, value):
        def isint(string):
            try:
                val = int(string)
                return True
            except ValueError:
                return False
        
        def isfloat(string):
            try:
                val = float(string)
                return True
            except ValueError:
                return False

        if value == 'true':
            return True
        elif value == 'false':
            return False
        elif value == "None":
            return None
        elif isint(value):
            return int(value)
        elif isfloat(value):
            return float(value)
        else:
            return value

    @TL_request
    def __getitem__(self, infoName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to access info "{infoName}" value')
        if infoName not in self.infoList:
            # Logging the exception
            self.logger.error(f'Info name "{infoName}" does not exist')
            raise Exception('Info name does not exist')
        
        sendCommand(self, f'getinfo {infoName}')
        readString = findLine(self, f'Info: {infoName}')
        idx_start = len(f'Info: {infoName}')
        infoValue = trimmer(readString, start=idx_start, length=1)

        # NB you don't need to set pointer to 'EndInputQuery' because
        # such message does not exists.
        return self.__adjustValue(infoValue)

    @TL_request
    def __setitem__(self, infoName, newValue):
        # Logging the setitem attempt
        self.logger.info(f'Attempting to set info "{infoName}" to "{newValue}"')
        if infoName not in self.infoList:
            # Logging the exception
            self.logger.error(f'Info name "{infoName}" does not exist')
            raise Exception('Info name does not exist')

        sendCommand(self, f'updateInfo {infoName} "{newValue}"')
        findLine(self, f'Updated info for {infoName}')
    
    @TL_request
    def __repr__(self):
        infoList = self.infoList

        fullCommand = ''
        for info in infoList:
            fullCommand += f'getinfo {info}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'inputList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)

        description = 'Info: \r\n'
        for info in infoList:
            readString = findLine(self, f'Info: {info}')
            idx_start = len(f'Info: {info}')
            infoValue = trimmer(readString, start=idx_start, length=1)

            description += f'    {info}: {infoValue}\r\n'
        
        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, infoName):
        if infoName in self.infoList:
            return True
        else:
            return False
    
    def __iter__(self):
        for infoName in self.infoList:
            yield infoName

# This way of defining dictionaries allows for property laws to work
# correctly when retrieving data.
class ParameterDict(dict):
    '''Dictionary of parameters that access the value property when accessing items.
    '''
    def __init__(self, dictionary, name):
        super().__init__(dictionary)
        self.name = name

    def __getitem__(self, key):
        if key in self:
            return dict.__getitem__(self, key).value
        else:
            raise Exception('Parameter name does not exist')

    def __setitem__(self, key, newValue):
        if key in self:
            dict.__getitem__(self, key).value = newValue
        else:
            pass

class ConfigSectionDict(dict):
    '''Class that acts like a dictionary of configuration sections.
    
    This class acts like a dictionary, returning ParameterDict container objects 
    full of :py:class:`~AMRBase.ConfigParameter` objects that fully describes an AMR or FM full config.
    This class it's not meant to be used by the user, instead it is meant to be
    used by the config property in the :py:class:`~AMRBase.FleetManager` and :py:class:`~AMRBase.Robot` objects.

    Attributes:
        configSectionList (:py:class:`list` of :py:class:`str`): List containing the names of all the sections
            in the AMR or FM configuration.
        
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
        

    '''
    def __init__(self, configSectionList, TL_obj, TL_requests, logger = generalLog):
        '''
        Args:
            configSectionList (:py:class:`list` of :py:class:`str`): List containing the names of all the sections
                in the AMR or FM configuration.

            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
        
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.configSectionList = configSectionList
        self.TL_obj = TL_obj
        self.TL_requests = TL_requests
        self.logger = logger
    
    @TL_request
    def __getitem__(self, sectionName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to request config section "{sectionName}" values')
        if sectionName not in self.configSectionList:
            # Logging the exception
            self.logger.error(f'Config section "{sectionName}" not in config section list')
            raise Exception(f'Config section "{sectionName}" not in config section list')
        
        sendCommand(self, f'getConfigSectionValues {sectionName}\n')

        paramList = []
        valueList = []
        while True:
            readString = findLine(self, 'GetConfigSectionValue')
            if 'EndOfGetConfigSectionValues' in readString:
                break
            else:
                tempParam = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
                if tempParam:
                    paramList.append(tempParam)
                    valueList.append(trimmer(readString, find_nth(readString, ' ', 2), 1))

        tempParamdict = {}
        for parameter, value in zip(paramList, valueList):
            tempParamdict[parameter] = ConfigParameter(self.TL_obj, parameter, sectionName, value, self.TL_requests, self.logger)
        
        return ParameterDict(tempParamdict, sectionName)
        
    def __setitem__(self, key, newValue):
        pass
    
    @TL_request
    def __repr__(self):
        configSectionList = self.configSectionList
        fullCommand = ''
        for section in configSectionList:
            fullCommand += f'getConfigSectionValues {section}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'configSectionList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)

        description = ''
        for section in configSectionList:
            paramList = []
            valueList = []
            while True:
                readString = findLine(self, 'GetConfigSectionValue')
                if 'EndOfGetConfigSectionValues' in readString:
                    break
                else:
                    tempParam = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
                    if tempParam:
                        paramList.append(tempParam)
                        valueList.append(trimmer(readString, find_nth(readString, ' ', 2), 1))

            description += f'Section: {section}\r\n'
            for parameter, value in zip(paramList, valueList):
                description += f'   {parameter}: {value}\r\n'
        
        # Delete last unnecessary '\r\n'
        description = description[:-2]
        return description

    # Following methods are implemented to mimic a dictionary
    def __contains__(self, sectionName):
        if sectionName in self.configSectionList:
            return True
        else:
            return False
        
    def __iter__(self):
        for sectionName in self.configSectionList:
            yield self.__getitem__(sectionName)

class DatastoreGroupDict(dict):
    '''Class that acts like a dictionary of datastore groups.
    
    This class acts like a dictionary, returning :py:class:`~AMRBase.ParameterDict` container objects 
    full of :py:class:`~AMRBase.Parameter` objects that fully describes an AMR or FM full datastore.
    This class it's not meant to be used by the user, instead it is meant to be
    used by the datastore property in the :py:class:`~AMRBase.FleetManager` and :py:class:`~AMRBase.Robot` objects.

    Attributes:
        datastoreGroupList (:py:class:`list` of :py:class:`str`): List containing the names of all the groups
            in the AMR or FM datastore.
        
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, datastoreGroupList, TL_requests, logger = generalLog):
        '''
        Args:
            datastoreGroupList (:py:class:`list` of :py:class:`str`): List containing the names of all the groups
                in the AMR or FM datastore.
        
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                    With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
            
            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.
            
            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.datastoreGroupList = datastoreGroupList
        self.TL_requests = TL_requests
        self.logger = logger

    @TL_request
    def __getitem__(self, groupName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to request datastore group "{groupName}" values')
        if groupName not in self.datastoreGroupList:
            # Logging the exception
            self.logger.error(f'Group "{groupName}" not in datastore group list')
            raise Exception(f'Group "{groupName}" not in datastore group list')

        sendCommand(self, f'getDataStoreGroupValues {groupName}')

        paramList = []
        valueList = []
        while True:
            readString = findLine(self, 'GetDataStoreGroupValues')
            if 'EndOfGetDataStoreGroupValues' in readString:
                break
            else:
                tempParam = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
                if bool(tempParam):
                    paramList.append(tempParam)
                    valueList.append(trimmer(readString, find_nth(readString, ' ', 2), 1))

        tempParamdict = {}
        for parameter, value in zip(paramList, valueList):
            tempParamdict[parameter] = Parameter(self.TL_obj, parameter, groupName, value, self.logger)
        
        return ParameterDict(tempParamdict, groupName)
    
    @TL_request
    def __repr__(self):
        datastoreGroupList = self.datastoreGroupList

        fullCommand = ''
        for group in datastoreGroupList:
            fullCommand += f'getDataStoreGroupValues {group}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'datastoreGroupList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)

        description = 'DataStore:\r\n'
        for group in datastoreGroupList:
            description += f'    Group: {group}\r\n'
            while True:
                readString = findLine(self, 'GetDataStoreGroupValues')
                if 'EndOfGetDataStoreGroupValues' in readString:
                    break

                paramName = trimmer(readString, find_nth(readString, ' ', 1), 1, find_nth(readString, ' ', 2))
                value = trimmer(readString, find_nth(readString, ' ', 2), 1)

                description += f'        {paramName}: {value}\r\n'

        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, groupName):
        if groupName in self.datastoreGroupList:
            return True
        else:
            return False
    
    def __iter__(self):
        for groupName in self.datastoreGroupList:
            yield self.__getitem__(groupName)

class Status:
    '''Class describing informations about the robot status.

    Attributes:
        aramStatus (:py:class:`str`): The status of the robot as described by ARAM
        
        stateOfCharge (:py:class:`float`): The robot internal (CPU) temperature.

        location (:py:class:`list` of :py:class:`int`): List containing in order X Y and Heading
            of the robot current position.
        
        loc_score (:py:class:`float`): The localization score of the robot. The value
            can go from 0 (worst) to 100 (best).
        
        temperature (:py:class:`float`): The CPU current temperature in Celsius degrees.
    '''
    def __init__(self, status_str, stateOfCharge, location, localizationScore, temperature):
        self.aramStatus = status_str
        self.stateOfCharge = stateOfCharge
        self.location = location
        self.loc_score = localizationScore
        self.temperature = temperature
    
    def __repr__(self):
        description = f'''
        Status:
            Aram Status:         {self.aramStatus}
            State of Charge:     {self.stateOfCharge}

            Location:            X: {self.location[0]} [mm], Y: {self.location[1]} [mm], Θ: {self.location[2]} [deg]
            Localization Score:  {self.loc_score}

            CPU Temperature:     {self.temperature}
        '''
        return inspect.cleandoc(description)

class InputDict(dict):
    '''Class that acts like a dictionary of input values.

    This class acts like a dictionary, returning input values when items are accessed.
    Input values are returned as booleans, True for on and False for off. The single
    input values can also be set, but this only works if the input value is actually
    an output having the type set as "customAlsoInput" in the AMR configuration.
    This class is not meant to be used by the user, instead it is meant to be
    accessed by the :py:attr:`~AMRBase.Robot.inputs` property of the :py:class:`~AMRBase.Robot` objects.

    Attributes:
        inputList (:py:class:`list` of :py:class:`str`): List containing all the names of the inputs.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
            
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, inputList, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            inputList (:py:class:`list` of :py:class:`str`): List containing all the names of the inputs.
        
            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.

            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.inputList = inputList
        self.TL_requests = TL_requests
        self.logger = logger

    @TL_request
    def __getitem__(self, inputName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to access input "{inputName}" value')
        if inputName not in self.inputList:
            # Logging the exception
            self.logger.error(f'Input name "{inputName}" does not exist')
            raise Exception('Input name does not exist')
        
        sendCommand(self, f'inputquery {inputName}')
        readString = findLine(self, f'Input: {inputName}')
        inputValue = trimmer(readString, find_nth(readString, ' ', 2), 1)

        # NB you don't need to set pointer to 'EndInputQuery' because
        # such message does not exists.
        return onoffToBool(inputValue)

    @TL_request
    def __setitem__(self, inputName, value):
        # Logging the setitem attempt
        self.logger.info(f'Attempting to set input "{inputName}" value')
        if inputName not in self.inputList:
            # Logging the exception
            self.logger.error(f'Input name "{inputName}" does not exist')
            raise Exception('Input name does not exist')

        if value:
            sendCommand(self, f'outputon {inputName}')
        else:
            sendCommand(self, f'outputoff {inputName}')
    
    @TL_request
    def __repr__(self):
        inputList = self.inputList

        fullCommand = ''
        for input in inputList:
            fullCommand += f'inputquery {input}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'inputList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)

        description = 'Inputs: \r\n'
        for input in inputList:
            readString = findLine(self, f'Input: {input}')
            inputValue = trimmer(readString, find_nth(readString, ' ', 2), 1)

            description += f'    {input}: {inputValue}\r\n'
        
        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, inputName):
        if inputName in self.inputList:
            return True
        else:
            return False
    
    def __iter__(self):
        for inputName in self.inputList:
            yield inputName

class OutputDict(dict):
    '''Class that acts like a dictionary of output values.

    This class acts like a dictionary, returning output values when items are accessed.
    Output values are returned as booleans, True for on and False for off. The single
    output values can also be set, and this affects the actual state of the output
    in the AMR.
    This class is not meant to be used by the user, instead it is meant to be
    accessed by the :py:attr:`~AMRBase.Robot.outputs` property of the :py:class:`~AMRBase.Robot` objects.

    Attributes:
        outputList (:py:class:`list` of :py:class:`str`): List containing all the names of the outputs.

        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
            With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        logger (:py:class:`~logging.Logger`): The logger to log to.
    '''
    def __init__(self, TL_obj, outputList, TL_requests, logger = generalLog):
        '''
        Args:
            TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
                With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.

            outputList (:py:class:`list` of :py:class:`str`): List containing all the names of the outputs.
        
            TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
                access to the Telnet server.

            logger (:py:class:`~logging.Logger`): The logger to log to.
        '''
        self.TL_obj = TL_obj
        self.outputList = outputList
        self.TL_requests = TL_requests
        self.logger = logger

    @TL_request
    def __getitem__(self, outputName):
        # Logging the getitem attempt
        self.logger.info(f'Attempting to access output "{outputName}" value')
        if outputName not in self.outputList:
            # Logging the exception
            self.logger.error(f'Input name "{outputName}" does not exist')
            raise Exception('Output name does not exist')
        
        sendCommand(self, f'outputquery {outputName}')
        readString = findLine(self, f'Output: {outputName}')
        outputValue = trimmer(readString, find_nth(readString, ' ', 2), 1)

        # NB you don't need to set pointer to 'EndInputQuery' because
        # such message does not exists.
        return onoffToBool(outputValue)

    @TL_request
    def __setitem__(self, outputName, value):
        # Logging the setitem attempt
        self.logger.info(f'Attempting to set output "{outputName}" value')
        if outputName not in self.outputList:
            # Logging the exception
            self.logger.error(f'Input name "{outputName}" does not exist')
            raise Exception('Output name does not exist')

        if value:
            sendCommand(self, f'outputon {outputName}')
        else:
            sendCommand(self, f'outputoff {outputName}')
    
    @TL_request
    def __repr__(self):
        outputList = self.outputList

        fullCommand = ''
        for output in outputList:
            fullCommand += f'outputquery {output}\n '
        # Remove last un-needed space from the full command list
        fullCommand = fullCommand[:-1]
        # Avoid sending command if 'outputList' is empty, which would result in
        # an empty command string ""
        if fullCommand:
            sendCommand(self, fullCommand)

        description = 'Outputs: \r\n'
        for output in outputList:
            readString = findLine(self, f'Output: {output}')
            outputValue = trimmer(readString, find_nth(readString, ' ', 2), 1)

            description += f'    {output}: {outputValue}\r\n'
        
        return description
    
    # Following methods are implemented to mimic a dictionary
    def __contains__(self, outputName):
        if outputName in self.outputList:
            return True
        else:
            return False
    
    def __iter__(self):
        for outputName in self.outputList:
            yield outputName

class RobotDict(dict):
    '''This class is just a dictionary class with custom __repr__.

    This class is used by the :py:class:`~AMRBase.FleetManager`'s :py:attr:`~AMRBase.robots` attribute, which is a dictionary
    of :py:class:`~AMRBase.Robot` objects. It's exactly equal to a dictionary with a custom representation
    method to make it readeable by humans.
    '''
    def __repr__(self):
        description = 'Robots:\r\n'
        for key in self:
            description += f'    {key}: Robot {self[key].IP_address}\r\n'
        return description
            

#%% Main Classes Definiton

class MasterAMR:
    '''Parent class of :py:class:`~AMRBase.Robot` and :py:class:`~AMRBase.FleetManager`, used for inheritance purposes.

    This class was create to store all properties and methods common for both the
    user classes ':py:class:`~AMRBase.Robot`' and ':py:class:`~AMRBase.FleetManager`'. Everything that can be used by both an
    EM or AMR is found inside here instead of duplicating everything in the two
    classes. Therefore, this class is absolutely not meant to be used by the user,
    instead it's just a parent class that is used for practicality purposes.

    Attributes:
        IP_address (:py:class:`str`): The IP address of the appliance, inserted as a string.

        password (:py:class:`str`): The user-created password of the ARCL Telnet server, as
            defined in MobilePlanner.
            **Default**: adept
        
        comm_port (:py:class:`int`): The port used by the appliance to communicate through ARCL.
            **Default**: 7171
        
        autostart (:py:class:`bool`): Flag that defines whether or not to "autostart" the object
            when the class is instantiated. It's True by default, setting it to
            False does not start the Telnet communication at object instantiation and
            should only be used for debugging purposes.
            **Default**: True

        TL_requests (:py:class:`list` of :py:class:`Thread`): List of threads that are requesting
            access to the Telnet server.
        
        TL_obj (:py:class:`~telnetlib.Telnet`): Telnet server to send the command to.
	        With PyARCL this can be done through the :py:func:`~AMRBase.startServer` function.
        
        logger (:py:class:`~logging.Logger`): The logger to log to. This is generated
            as the object is instantiated and is unique per-object.
        
        logLevel (:py:class:`str`): String that represents the log level requested for the object.
            Possible values are 'debug', 'info', 'warning', 'error', 'critical'.
            **Default**: INFO level.
    '''
    def __init__(self, IP, password='adept', port=7171, autostart=True, logLevel = 'info'):
        '''
        Args:
            IP (:py:class:`str`): The IP address of the appliance, inserted as a string.

            password (:py:class:`str`): The user-created password of the ARCL Telnet server, as
                defined in MobilePlanner.
                **Default**: adept
            
            port (:py:class:`int`): The port used by the appliance to communicate through ARCL.
                **Default**: 7171
            
            autostart (:py:class:`bool`): Flag that defines whether or not to "autostart" the object
                when the class is instantiated. It's True by default, setting it to
                False does not start the Telnet communication at object instantiation and
                should only be used for debugging purposes.
                **Default**: True
            
            logLevel (:py:class:`str`): String that represents the log level requested for the object.
                Possible values are 'debug', 'info', 'warning', 'error', 'critical'.
                **Default**: INFO level.
        '''
        self.IP_address = IP
        self.password = password
        self.comm_port = port
        self.logLevel = logLevel

        # TL_request is the list of threads that asked access to the Telnet
        # textserver. Those thread need to be generated by the TelnetRequestThread
        # class and wait for each other to finish before accessing the textserver,
        # allowing for stable, multi-access read management.
        self.TL_requests = ThreadSafeList()

        if autostart:
            self.__start()

    def __start(self):
        self.TL_obj = startServer(self.IP_address, self.password, self.comm_port)

        self._setinitialValues_()
        # Logging the Telnet connection confirmation
        self.logger.info(f'Telnet connection with {self.TL_obj.host} started')

        findLine(self, 'End of commands\r\n')

    # The 'setinitialvalues' method is used by the MasterAMR subclasses to declare shared 
    # lists and variables used by other methods
    def _setinitialValues_(self):
        self.logger = generalLog

    @TL_request
    def __getgoalList(self):
        sendCommand(self, 'getgoals')
        goalList = []
        # the 'readuntil' function cannot be used here since 'Goal:' and
        # 'End of goals' do not share a common key.
        # This unfortunately generates a possible robustness problem if 'getgoals'
        # command is not interpreted correctly by the object, since using 'readLine'
        # instead of 'findLine' means that no exception will be raised in case of
        # "Unknown command" or "CommandError" ARCL errors.
        # NB no longer true, exception handling has been implemented here
        while True:
            readString = readLine(self.TL_obj, self.logger)
            if 'Goal:' in readString:
                goalList.append(trimmer(readString))
            elif readString == 'End of goals\r\n':
                break

            # Exception Handling...
            elif 'Unknown command' in readString:
                commandName = readString[16:-2]
                self.logger.error(f'goalList function read "{commandName}" unknown command exception')
                raise Exception(f'Read "Unknown command {commandName}"')
            elif 'CommandError' in readString:
                commandName = readString[14:-2]
                err_desc_line = readLine(self.TL_obj, self.logger)
                if 'CommandErrorDescription' in err_desc_line:
                    err_desc = err_desc_line[25:-2]
                    print(err_desc_line)
                    self.logger.error(f'goalList function read error related to "{commandName}", error description says "{err_desc}"')
                    raise Exception(f'Command "{commandName}" returns error "{err_desc}"')
                else:
                    self.logger.error(f'goalList function read error related to "{commandName}"')
                    raise Exception(f'Command "{commandName}" returns error')
        
        return goalList

    @property
    def goalList(self):
        ''':py:class:`list` of :py:class:`str`: List containing the names of all the goals present in map.
        '''
        # Logging the goalList request
        self.logger.info(f'Requested goalList')
        goalList = self.__getgoalList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(goalList))}')
        return goalList

    @TL_request
    def queueMulti(self, missionGoalList, typeList = None, priorityList = None, customJobId = None):
        '''General-purpose method to generate complex jobs.

        Args:
            missionGoalList (:py:class:`list` of :py:class:`str`): list containing the goal sequence
                of the needed job segments. The list can contain any number of
                goals, with an upper limit of NEED_TO_CHECK.
                For easiness of use, this parameter can be inserted as a simple
                string; if that is the case, a simple pickup job is created.
            
            typeList (:py:class:`list` of :py:class:`str`): list containing the job segment type for
                each requested goal. the job segment type can be either 'Pickup'
                or 'Dropoff'.
                For easiness of use, this parameter can be inserted as a simple
                string; if that is the case, all job segments will have the specified type
                except the first which will always be a pickup job segment.
                **Default**: If typeList is not specified, the job segment types
                are assumed to alternate between pickup and dropoff for all
                job segments.
                
            priorityList (:py:class:`list` of :py:class:`int`): list containing the priorities for
                each job segment.
                For easiness of use, this parameter can be inserted as a simple
                int; if that is the case, all job segments will have the specified priority.
                **Default**: If priorityList is not specified, the priorities for
                all job segment types are assumed to be the default as
                defined in the config.
            
            customJobId (:py:class:`str`): the custom id to give to the job.
                **Default**: The default names ARAM assigns to the jobs.
        '''
        # Logging the queueMulti attempt
        self.logger.info(f'Attempting to use queueMulti for the goals {slm(str(missionGoalList))}')
        try:
            queuingManager = self.config['Queuing Manager']
        except KeyError:
            # Logging the exception
            self.logger.error('Robot or FleetManager does not have Queuing Manager active')
            raise Exception('Robot or FleetManager does not have Queuing Manager active')

        defaultPickupPriority = queuingManager['DefaultPickupPriority']
        defaultDropoffPriority = queuingManager['DefaultDropoffPriority']

        if type(missionGoalList) is str:
            missionGoalList = [missionGoalList]
        elif (type(missionGoalList) is list) & all(isinstance(goal, str) for goal in missionGoalList):
            pass
        else:
            # Logging the exception
            self.logger.error('"missionGoalList" parameter must be either a list of strings or a string')
            raise Exception('"missionGoalList" parameter must be either a list of strings or a string')
        

        # For speed reasons 'self.goalList' is assigned by reference to 'goalList'
        # local variable.
        goalList = self.goalList
        if not all(missionGoal in goalList for missionGoal in missionGoalList):
            # Logging the exception
            self.logger.error('Some or all mission goals are not present in map')
            raise Exception('Some or all mission goals are not present in map')

        n_goals = len(missionGoalList)
        if typeList == None:
            # if typeList is not specified, then the order of pickup-dropoff
            # operations is chosen arbitrarily to be pickup-dropoff-pickup-dropoff
            # et cetera.
            typeList = ['Pickup' if i % 2 == 0 else 'Dropoff' for i in range(0, n_goals)]
        if priorityList == None:
            priorityList = [defaultPickupPriority if item.capitalize() == 'Pickup' else defaultDropoffPriority for item in typeList]
        

        if type(priorityList) is int:
            priorityList = [priorityList for goal in missionGoalList]
        elif (type(priorityList) is list) & all(isinstance(priority, int) for priority in priorityList):
            pass
        else:
            # Logging the exception
            self.logger.error('"priorityList" parameter must be either a list of ints or an int')
            raise Exception('"priorityList" parameter must be either a list of ints or an int')

        if type(typeList) is str:
            typeList = [typeList for goal in missionGoalList]
            typeList[0] = 'Pickup' # First segment will always need to be a pickup
        elif (type(typeList) is list) & all(isinstance(jobtype, str) for jobtype in typeList):
            pass
        else:
            # Logging the exception
            self.logger.error('"typeList" parameter must be either a list of strings or a string')
            raise Exception('"typeList" parameter must be either a list of strings or a string')


        if len(typeList) != n_goals & len(priorityList) != n_goals:
            # Logging the exception
            self.logger.error('Error: Vector lengths do not match')
            raise Exception('Error: Vector lengths do not match')

        # Sending the actual command
        commandString = f'queuemulti {n_goals} 2'
        for missionGoal, typology, priority in zip(missionGoalList, typeList, priorityList):
            commandString += f' {missionGoal} {typology} {str(priority)}'
        if customJobId:
            commandString += f' {customJobId}'
        
        sendCommand(self, commandString)
        
        # Reading feedback and using it to generate a dictionary of 'JobSegment' objects
        idList = []
        while True:
            readString = findLine(self, 'QueueMulti')
            if readString == 'EndQueueMulti\r\n':
                break
            id = trimmer(readString, find_nth(readString, ' ', 7), 1, find_nth(readString, ' ', 8))
            idList.append(id)
            jobid = trimmer(readString, find_nth(readString, ' ', 10), 1, find_nth(readString, ' ', 11))

        # Note how 'self.goalList' and 'self.TL_requests' are passed by reference:
        # - 'goalList' will be always updated as it calls the property of the original object
        # - 'TL_requests' for the @TL_request decorator will be the one of the original object (!!!)
        
        tempDict = JobSegmentDict(self.TL_obj, idList, jobid, goalList, False, self.TL_requests, self.logger)
        
        return Job(self.TL_obj, idList, jobid, tempDict, False, self.TL_requests, self.logger)

    @TL_request
    def queuePickup(self, missionGoal, priority = None, customJobId = None):
        '''Generates a pickup mission to the AMR/FM and returns the :py:class:`~AMRBase.Job` object 
        related to said mission.

        Args:
            missionGoal (:py:class:`str`): The mission goal the pickup job segment should
                be requesting.
        
            priority (:py:class:`int`): The priority of the pickup job segment.
                **Default**: The default pickup priority as defined in config.
            
            customJobId (:py:class:`str`): the custom id to give to the job.
                **Default**: The default names ARAM assigns to the jobs.
        '''
        # Logging the queuePickup attempt
        self.logger.info(f'Attempting to use queuePickup for the goal "{missionGoal}"')
        try:
            defaultPickupPriority = self.config['Queuing Manager']['DefaultPickupPriority']
        except KeyError:
            # Logging the exception
            self.logger.error('Robot or FleetManager does not have Queuing Manager active')
            raise Exception('Robot or FleetManager does not have Queuing Manager active')

        if not type(missionGoal) == str:
            # Logging the exception
            self.logger.error('"missionGoal" parameter must be a string')
            raise Exception('"missionGoal" parameter must be a string')

        # For speed reasons 'self.goalList' is assigned by reference to 'goalList'
        # local variable.
        goalList = self.goalList
        if missionGoal not in goalList:
            # Logging the exception
            self.logger.error(f'Goal "{missionGoal}" not in map')
            raise Exception(f'Goal "{missionGoal}" not in map')

        commandString = f'queuepickup {missionGoal}'
        if priority:
            commandString += f' {priority}'
        if customJobId:
            if not priority:
                 commandString += f' {defaultPickupPriority}'
            commandString += f' {customJobId}'
        sendCommand(self, commandString)
        
        # Reading feedback and using it to generate a dictionary of 'JobSegment' objects

        readString = findLine(self, 'queuepickup')

        id = trimmer(readString, find_nth(readString, ' ', 7), 1, find_nth(readString, ' ', 8))
        idList = [id]
        jobid = trimmer(readString, find_nth(readString, ' ', 10), 1, find_nth(readString, ' ', 11))

        tempDict = JobSegmentDict(self.TL_obj, idList, jobid, goalList, False, self.TL_requests, self.logger)

        return Job(self.TL_obj, idList, jobid, tempDict, False, self.TL_requests, self.logger)
    
    @TL_request
    def queuePickupDropoff(self, missionGoal1, missionGoal2, priority1 = None, priority2 = None, customJobId = None):
        '''Generates a simple pickup-dropoff mission to the AMR/FM and returns the
        :py:class:`~AMRBase.Job` object related to said mission.

        Args:
            missionGoal1 (:py:class:`str`): The mission goal related to the pickup 
                job segment.

            missionGoal2 (:py:class:`str`): The mission goal related to the dropoff 
                job segment.
            
            priority1 (:py:class:`int`): The priority of the pickup job segment.
                **Default**: The default pickup priority as defined in config.
            
            priority2 (:py:class:`int`): The priority of the dropoff job segment.
                **Default**: The default dropoff priority as defined in config.
            
            customJobId (:py:class:`str`): the custom id to give to the job.
                **Default**: The default names ARAM assigns to the jobs.
        '''
        # Logging the queuePickupDropoff attempt
        self.logger.info(f'Attempting to use queuePickupDropoff for the goals "{missionGoal1}" and "{missionGoal2}"')
        try:
            queuingManager = self.config['Queuing Manager']
        except KeyError:
            # Logging the exception
            self.logger.error('Robot or FleetManager does not have Queuing Manager active')
            raise Exception('Robot or FleetManager does not have Queuing Manager active')

        defaultPickupPriority = queuingManager['DefaultPickupPriority']
        defaultDropoffPriority = queuingManager['DefaultDropoffPriority']

        if not ((type(missionGoal1) == str) & (type(missionGoal2) == str)):
            # Logging the exception
            self.logger.error('Both "missionGoal" parameters must be a string')
            raise Exception('Both "missionGoal" parameters must be a string')

        goalList = self.goalList
        if (missionGoal1 not in goalList) & (missionGoal2 not in goalList):
            # Logging the exception
            self.logger.error(f'Either or both mission goals are not in map')
            raise Exception(f'Either or both mission goals are not in map')

        commandString = f'queuepickupdropoff {missionGoal1} {missionGoal2}'
        # This time both priorities need to be specified or we wouldn't
        # have the possibility to only customize one of them.
        if priority1:
            commandString += f' {priority1}'
        else:
            commandString += f' {defaultPickupPriority}'
        if priority2:
            commandString += f' {priority2}'
        else:
            commandString += f' {defaultDropoffPriority}'
        if customJobId:
            commandString += f' {customJobId}'
        sendCommand(self, commandString)
        
        # Reading feedback and using it to generate a dictionary of 'JobSegment' objects
        readString = findLine(self, 'queuepickupdropoff')

        id1 = trimmer(readString, find_nth(readString, ' ', 11), 1, find_nth(readString, ' ', 12))
        id2 = trimmer(readString, find_nth(readString, ' ', 13), 1, find_nth(readString, ' ', 14))
        idList = [id1, id2]
        jobid = trimmer(readString, find_nth(readString, ' ', 15), 1, find_nth(readString, ' ', 16))

        tempDict = JobSegmentDict(self.TL_obj, idList, jobid, goalList, False, self.TL_requests, self.logger)

        return Job(self.TL_obj, idList, jobid, tempDict, False, self.TL_requests, self.logger)

    @TL_request
    def __getqueueList(self):
        commandString = 'queueshow'
        sendCommand(self, commandString)

        queueList = []
        while True:
            readString = findLine(self, 'QueueShow')
            if readString == 'EndQueueShow\r\n':
                break
            jobid = trimmer(readString, find_nth(readString, ' ', 2), 1, find_nth(readString, ' ', 3))
            if jobid not in queueList:
                queueList.append(jobid)
        
        return queueList
    
    @property
    def queueList(self):
        ''':py:class:`list` of :py:class:`str`: List containing the names of all the jobs in the queue;
        whether those are in progress, completed or cancelled.
        '''
        # Logging the queueList request
        self.logger.info(f'Requested queueList')
        queueList = self.__getqueueList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(queueList))}')
        return queueList
    
    @TL_request
    def __getqueue(self):
        queueList = self.queueList
        goalList = self.goalList

        return JobDict(self.TL_obj, queueList, goalList, self.TL_requests, self.logger)

    @property
    def queue(self):
        ''':py:class:`~AMRBase.JobDict`: Container acting like a dictionary of :py:class:`~AMRBase.JobDict` objects.
        Keys are the unique ids of the jobs, which can be accessed through the
        :py:attr:`~AMRBase.MasterAMR.queueList` property. Allows iteration.
        '''
        # Logging the queueList request
        self.logger.info(f'Requested queue')
        queue = self.__getqueue()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(queue))}')
        return queue
    
    @TL_request
    def __getconfigSectionList(self):
        sendCommand(self, 'getConfigSectionList')

        configSectionList = []
        while True:
            readString = findLine(self, 'GetConfigSectionList')
            if 'EndOfGetConfigSectionList' in readString:
                break
            else:
                configSectionList.append(trimmer(readString))
        
        return configSectionList

    @property
    def configSectionList(self):
        ''':py:class:`list` of :py:class:`str`: List containing the names of all the sections
        in the AMR or FM configuration.
        '''
        # Logging the configSectionList request
        self.logger.info(f'Requested configSectionList')
        configSectionList = self.__getconfigSectionList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(configSectionList))}')
        return configSectionList
    
    # Great part of this code was shamelessly stripped from "__getqueue"
    def __getconfig(self):         
        configSectionDict = ConfigSectionDict(self.configSectionList, self.TL_obj, self.TL_requests, self.logger)
        return configSectionDict
    
    @property
    def config(self):
        ''':py:class:`~AMRBase.ConfigSectionDict`: Container acting like a dictionary of 
        :py:class:`~AMRBase.ParameterDict` objects. Keys are the names of the sections, which can
        be accessed through the :py:class:`~AMRBase.MasterAMR.configSectionList` property. Allows iteration.
        '''
        # Logging the config request
        self.logger.info(f'Requested config')
        config = self.__getconfig()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(config))}')
        return config
    
    @TL_request
    def __getdatastoreGroupList(self):
        sendCommand(self, 'getDataStoreGroupList')

        configSectionList = []
        while True:
            readString = findLine(self, 'GetDataStoreGroupList')
            if 'EndOfGetDataStoreGroupList' in readString:
                break
            else:
                configSectionList.append(trimmer(readString))
        
        return configSectionList
    
    @property
    def datastoreGroupList(self):
        ''':py:class:`list` of :py:class:`str`: List containing the names of all the groups in the AMR 
        or FM datastore.
        '''
        # Logging the datastoreGroupList request
        self.logger.info(f'Requested datastoreGroupList')
        datastoreGroupList = self.__getdatastoreGroupList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(datastoreGroupList))}')
        return datastoreGroupList
            
    def __getdatastore(self):
        datastoreGroupDict = DatastoreGroupDict(self.TL_obj, self.datastoreGroupList, self.TL_requests, self.logger)
        return datastoreGroupDict

    @property
    def datastore(self):
        ''':py:class:`~AMRBase.DatastoreGroupDict`: Container acting like a dictionary of 
        :py:class:`~AMRBase.ParameterDict` objects. Keys are the names of the groups, which can
        be accessed through the :py:class:`~AMRBase.MasterAMR.datastoreGroupList` property. Allows iteration.
        '''
        # Logging the datastore request
        self.logger.info(f'Requested datastore')
        datastore = self.__getdatastore()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(datastore))}')
        return datastore
    
    @TL_request
    def __getinfoList(self):
        sendCommand(self, 'getInfoList')
        infoList = []
        # the 'readuntil' function cannot be used here since 'InfoList:' and
        # 'End of info list' do not share a common key.
        # This unfortunately generates a possible robustness problem if 'getinfoList'
        # command is not interpreted correctly by the object, since using 'readLine'
        # instead of 'findLine' means that no exception will be raised in case of
        # "Unknown command" or "CommandError" ARCL errors.
        # NB no longer true, exception handling has been implemented here
        while True:
            readString = readLine(self.TL_obj, self.logger)
            if 'InfoList:' in readString:
                infoList.append(trimmer(readString))
            elif readString == 'End of info list\r\n':
                break

            # Exception Handling...
            elif 'Unknown command' in readString:
                commandName = readString[16:-2]
                self.logger.error(f'infoList function read "{commandName}" unknown command exception')
                raise Exception(f'Read "Unknown command {commandName}"')
            elif 'CommandError' in readString:
                commandName = readString[14:-2]
                err_desc_line = readLine(self.TL_obj, self.logger)
                if 'CommandErrorDescription' in err_desc_line:
                    err_desc = err_desc_line[25:-2]
                    print(err_desc_line)
                    self.logger.error(f'infoList function read error related to "{commandName}", error description says "{err_desc}"')
                    raise Exception(f'Command "{commandName}" returns error "{err_desc}"')
                else:
                    self.logger.error(f'infoList function read error related to "{commandName}"')
                    raise Exception(f'Command "{commandName}" returns error')
        
        return infoList

    @property
    def infoList(self):
        ''':py:class:`list` of :py:class:`str`: List containing the names of all the info names in the AMR 
        or FM info list.
        '''
        # Logging the datastoreGroupList request
        self.logger.info(f'Requested infoList')
        infoList = self.__getinfoList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(infoList))}')
        return infoList
    
    def __getinfo(self):
        return InfoDict(self.TL_obj, self.infoList, self.TL_requests, self.logger)

    @property
    def info(self):
        ''':py:class:`~AMRBase.InfoDict`: Container acting as a dictionary of info values.
        
        Keys are the info names, which can be accessed through the :py:attr:`~AMRBase.MasterAMR.infoList`
        property. Allows iteration.
        '''
        # Logging the inputs request
        self.logger.info(f'Requested info')
        info = self.__getinfo()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(info))}')
        return info
    
    @TL_request
    def createInfo(self, newInfoName, maxLength=20, startingValue='None'):
        '''Creates an info, accessible through the :py:attr:`~AMRBase.MasterAMR.info` property.

        Args:
            newInfoName (:py:class:`str`): The new info name.
                It needs not to contain any spaces, use '_' instead.

            maxLength (:py:class:`int`): The new info character length limit.
                **Default**: 20
            
            startingValue (:py:class:`str`): The starting value of the new info.
                It needs not to contain any spaces, use '_' instead.
                **Default**: 'None'
        '''
        self.logger.info(f'Attempting to create new info of name "{newInfoName}"')
        if ' ' in newInfoName:
            # Logging the exception
            self.logger.error(f'"Info name {newInfoName}" contains spaces, which is not supported')
            raise Exception(f'Info name {newInfoName} contains spaces, which is not supported; use "_" instead to separate words.')
        
        if ' ' in str(startingValue):
            # Logging the exception
            self.logger.error(f'Info starting value "{startingValue}" contains spaces, which is not supported')
            raise Exception(f'Info starting value {startingValue} contains spaces, which is not supported; use "_" instead to separate words.')
        
        if not isinstance(maxLength, int):
             # Logging the exception
            self.logger.error(f'Info max character length "{maxLength}" must be an integer')
            raise Exception(f'Info max character length must be an integer')
        elif maxLength < 0:
             # Logging the exception
            self.logger.error(f'Info max character length "{maxLength}" must be positive')
            raise Exception(f'Info max character length must be positive')

        sendCommand(self, f'createInfo {newInfoName} {maxLength} {startingValue}')
        findLine(self, f'Created info for {newInfoName}')

    @TL_request
    def __getdateTime(self):
        sendCommand(self, 'getdateTime')
        dateTime_str = trimmer(findLine(self, 'DateTime:'))
        return datetime.strptime(dateTime_str, '%m/%d/%Y %H:%M:%S')
    
    @property
    def dateTime(self):
        ''':py:class:`~datetime.datetime`: get current time of the client in DateTime
        time format.
        '''
        # Logging the dateTime request
        self.logger.info(f'Requested dateTime')
        RTC_time = self.__getdateTime()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(RTC_time))}')
        return RTC_time
    
    @TL_request
    def extIOInputUpdate(self, iofamily, io_number, state):
        '''Sends the edou command to set virtual input state.
        '''
        if state:
            statestring = '1'
        else:
            statestring = '0'

        sendCommand(self, f'edib {iofamily} {io_number} {statestring}')
        findLine(self, 'extIOInputUpdateBit')

    @TL_request
    def extIOOutputUpdate(self, iofamily, io_number, state):
        '''Sends the edou command to set virtual output state.
        '''
        if state:
            statestring = '1'
        else:
            statestring = '0'

        sendCommand(self, f'edob {iofamily} {io_number} {statestring}')
        findLine(self, 'extIOOutputUpdateBit')

    @TL_request
    def tripReset(self):
        '''Sends the tripReset command to the Robot / FleetManager object.
        '''
        sendCommand(self, 'tripreset')
        findLine(self, 'EndOfTripReset')



class FleetManager(MasterAMR):
    '''Class describing a Fleet Manager system.

    This class is the class the user should use to communicate with a Fleet Manager,
    usually running on an Enterprise Manager appliance. Use the FleetManager objects
    to access configuration, datastore, job queue and other useful data or to request
    new jobs and modify configuration parameters (Config change possible only on FC<2.1).

    Attributes:
        IP_address (:py:class:`str`): The IP address of the appliance, inserted as a string.

        password (:py:class:`str`): The user-created password of the ARCL Telnet server, as
            defined in MobilePlanner.
            **Default**: adept
        
        comm_port (:py:class:`int`): The port used by the appliance to communicate through ARCL.
            **Default**: 7171

        logLevel (:py:class:`str`): String that represents the log level requested for the object.
            Possible values are 'debug', 'info', 'warning', 'error', 'critical'.
            **Default**: INFO level.

        robots (:py:class:`dict` of :py:class:`~AMRBase.Robot`): by using the :py:meth:`~AMRBase.FleetManager.attachRobot` method one
            can add Robot objects to this dictionary, which indexes the Robot objects
            by their unique ids. Used to easy automation of access to the :py:class:`~AMRBase.Robot`
            objects on job-observation related scripts.
    '''

    # The 'robots' parameter can be used through the 'attachRobot' method
    # to enter them in a dictionary, so that they might be accessed through
    # their identifier.
    def _setinitialValues_(self):
        self.robots = RobotDict({})
        
        last_IP_digits = self.IP_address[find_nth(self.IP_address, '.', 3)+1:]
        self.logger = setupLogger(f'FleetManagerLog_{self.IP_address}', f'FM{last_IP_digits}.log', self.logLevel)

    def attachRobot(self, AMR):
        '''Attaches a :py:class:`~AMRBase.Robot` object to the robots dictionary attribute.

            Args:
                AMR (:py:class:`~AMRBase.Robot`): The :py:class:`~AMRBase.Robot` object to add to the :py:attr:`~AMRBase.FleetManager.robots` attribute.
        '''
        # Logging the robot attachment
        self.logger.info(f'Attempting to attach robot')
        if type(AMR) is not Robot:
            # Logging the exception
            self.logger.error('AMR argument is not a Robot object')
            raise Exception('AMR argument is not a Robot object')

        robotID = AMR.config['Enterprise Manager Connection']['Identifier']
        # Logging the attachment
        self.logger.info(f'Attempting to attach robot "{robotID}"')
        self.robots[robotID] = AMR

    def __repr__(self):
        versions = self.datastore['Versions']
        queueinformation = self.datastore['QueueInformation']
        jobcounts = self.datastore['JobCounts']
        fleetrobotinformation = self.datastore['FleetRobotInformation']

        description = f'''
        FleetManager: {self.IP_address}
            Connected Robots: {fleetrobotinformation['Robots']}

            Completed Jobs: {jobcounts['CompletedJobs']}
            Pending Jobs:   {queueinformation['QueueSizePending']}
            Cancelled Jobs: {jobcounts['CancelledJobs']}

            SetNetGo Ver.:  {versions['SNG']}
            ARAM Ver.:      {versions['ARAM']}
        '''
        return inspect.cleandoc(description)


class Robot(MasterAMR):
    '''Class describing an Autonomous Mobile Robot.

    This class is the class the user should use to communicate with an Autonomous
    Mobile Robot.
    Use the Robot objects to access status, IO, configuration, datastore, job queue
    and other useful data or to request new jobs, modify configuration parameters,
    force IO values, and command the Robot directly with methods like dock and goto.

    Attributes:
        IP_address (:py:class:`str`): The IP address of the appliance, inserted as a string.

        password (:py:class:`str`): The user-created password of the ARCL Telnet server, as
            defined in MobilePlanner.
            **Default**: adept
        
        comm_port (:py:class:`int`): The port used by the appliance to communicate through ARCL.
            **Default**: 7171

        logLevel (:py:class:`str`): String that represents the log level requested for the object.
            Possible values are 'debug', 'info', 'warning', 'error', 'critical'.
            **Default**: INFO level.
    '''

    def _setinitialValues_(self):        
        last_IP_digits = self.IP_address[find_nth(self.IP_address, '.', 3)+1:]
        self.logger = setupLogger(f'RobotLog_{self.IP_address}', f'RB{last_IP_digits}.log', self.logLevel)

    # Returns status...
    @TL_request
    def __getstatus(self):
        def str_to_num(string, type = 'float'):
            str_list = string.split(' ')
            if type == 'int':
                num_list = [int(i) for i in str_list]
            elif type == 'float':
                num_list = [float(i) for i in str_list]
            else:
                num_list = ['type specified not correct']
            if len(num_list) == 1:
                return num_list[0]
            else:
                return num_list
        
        sendCommand(self, 'status')
        
        status_str = trimmer(findLine(self, 'Status:'))
        stateOfCharge = str_to_num(trimmer(findLine(self, 'StateOfCharge:')))
        location = str_to_num(trimmer(findLine(self, 'Location:')), 'int')
        localizationScore = str_to_num(trimmer(findLine(self, 'LocalizationScore:')))
        temperature = str_to_num(trimmer(findLine(self,'Temperature:')))
        
        status_obj = Status(status_str, stateOfCharge, location, localizationScore, temperature)
        
        return status_obj
    
    @property
    def status(self):
        ''':py:attr:`~AMRBase.Status`: Returns a Status object containing informations
        about the robot status.
        '''
        # Logging the status request
        self.logger.info(f'Requested status')
        status = self.__getstatus()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(status))}')
        return status
    
    @TL_request
    def __getinputList(self):
            sendCommand(self, 'inputList')
            inputList = []
            while True:
                readString = findLine(self, 'InputList')
                if readString == 'End of InputList\r\n':
                    break
                inputList.append(trimmer(readString))
            return inputList
    
    @property
    def inputList(self):
        ''':py:class:`list` of :py:class:`str`: List containing all the names of the inputs.
        '''
        # Logging the inputList request
        self.logger.info(f'Requested inputList')
        inputList = self.__getinputList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(inputList))}')
        return inputList
    
    def __getinputs(self):
        return InputDict(self.TL_obj, self.inputList, self.TL_requests, self.logger)

    @property
    def inputs(self):
        ''':py:class:`~AMRBase.InputDict`: Container acting as a dictionary of input values.
        
        Keys are the input names, which can be accessed through the :py:attr:`~AMRBase.Robot.inputList`
        property. Allows iteration.
        '''
        # Logging the inputs request
        self.logger.info(f'Requested inputs')
        inputs = self.__getinputs()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(inputs))}')
        return inputs
    
    @TL_request
    def __getoutputList(self):
            sendCommand(self, 'outputList')
            outputList = []
            while True:
                readString = findLine(self, 'OutputList')
                if readString == 'End of OutputList\r\n':
                    break
                outputList.append(trimmer(readString))
            return outputList

    @property
    def outputList(self):
        ''':py:class:`list` of :py:class:`str`: List containing all the names of the outputs.
        '''
        # Logging the outputList request
        self.logger.info(f'Requested outputList')
        outputList = self.__getoutputList()
        # Logging the returned value
        self.logger.debug(f'Returned {slm(str(outputList))}')
        return outputList
    
    def __getoutputs(self):
        return OutputDict(self.TL_obj, self.outputList, self.TL_requests, self.logger)

    @property
    def outputs(self):
        '''Container acting as a dictionary of output values.

        Keys are the output names, which can be accessed through the :py:attr:`~AMRBase.Robot.outputList`
        property. Allows iteration. Outputs can be set to a boolean to force
        their value in the real robot.
        '''
        # Logging the outputs request
        self.logger.info(f'Requested outputs')
        outputs = self.__getoutputs()
        # Logging the returned value
        if self.logLevel == 'ultradebug':
            self.logger.debug(f'Returned {slm(str(outputs))}')
        return outputs
    
    @TL_request
    def queueDropoff(self, missionGoal, priority = None, customJobId = None):
        '''Generates a dropoff mission to the AMR/FM and returns the :py:class:`~AMRBase.Job` object 
        related to said mission. Note that this is the only queue command available
        to a robot connected to a Fleet Manager.

        Args:
            missionGoal (:py:class:`str`): The mission goal the dropoff job segment should
                be requesting.

            priority (:py:class:`int`): The priority of the dropoff job segment.
                **Default**: The default dropoff priority as defined in config.
            
            customJobId (:py:class:`str`): the custom id to give to the job.
                **Default**: The default names ARAM assigns to the jobs.
        '''
        # Logging the queueDropoff attempt
        self.logger.info(f'Attempting to use queueDropoff for the goal "{missionGoal}"')
        try:
            defaultDropoffPriority = self.config['Queuing Manager']['DefaultDropoffPriority']
            EM_connection = False
        except:
            # Check if Robot is connected to a Fleet Manager, skip exception in case
            try:
                EM_connection = self.config['Enterprise Manager Connection']['ConnectToEnterpriseManager']
                defaultDropoffPriority = 20 # Default dropoff priority of default config
            except:
                EM_connection = False

            if not EM_connection:
                # Logging the exception
                self.logger.error('Robot or FleetManager does not have Queuing Manager active')
                raise Exception('Robot or FleetManager does not have Queuing Manager active')

        if not type(missionGoal) == str:
            self.logger.error('"missionGoal" parameter must be a string')
            raise Exception('"missionGoal" parameter must be a string')

        # For speed reasons 'self.goalList' is assigned by reference to 'goalList'
        # local variable.
        goalList = self.goalList
        if missionGoal not in goalList:
            raise Exception(f'Goal "{missionGoal}" not in map')

        commandString = f'queuedropoff {missionGoal}'
        if priority:
            commandString += f' {priority}'
        if customJobId:
            if not priority:
                commandString += f' {defaultDropoffPriority}'
            commandString += f' {customJobId}'
        sendCommand(self, commandString)
        
        # 'queuedropoff' and only 'queuedropoff' generates this additional line
        # we must skip for the program to work correctly. That's just how
        # it is folks!
        readString = findLine(self, 'queuedropoff attempting to queue goal')
        # Reading feedback and using it to generate a dictionary of 'JobSegment' objects
        readString = findLine(self, 'queuedropoff')

        id = trimmer(readString, find_nth(readString, ' ', 7), 1, find_nth(readString, ' ', 8))
        idList = [id]
        jobid = trimmer(readString, find_nth(readString, ' ', 10), 1, find_nth(readString, ' ', 11))

        # Do not return anything if it was a EM-connected robot
        # NB Add a local version of the dicts so tht you can return objects in this case
        if EM_connection:
            tempDict = JobSegmentDict(self.TL_obj, idList, jobid, goalList, True, self.TL_requests, self.logger)
            return Job(self.TL_obj, idList, jobid, tempDict, True, self.TL_requests, self.logger)
        else:
            tempDict = JobSegmentDict(self.TL_obj, idList, jobid, goalList, False, self.TL_requests, self.logger)
            return Job(self.TL_obj, idList, jobid, tempDict, False, self.TL_requests, self.logger)

    # From now on, only simple or very simple commands are implemented

    @TL_request
    def goto(self, goal):
        '''
        Method that instructs the robot to go to a map goal.

        Args:
            goal (:py:class:`str`): The goal to go to. Will raise exception if goal is
                not in the list returned by the :py:attr:`~AMRBase.Robot.goalList` property (i.e. not in map).
        '''
        # Logging the goto command
        self.logger.info(f'Attempting to send robot to "{goal}"')
        if goal not in self.goalList:
            # Logging exception
            self.logger.error('Cannot go to goal: Goal name not in map')
            raise Exception('Cannot go to goal: Goal name not in map')
        sendCommand(self, f'goto {goal}')

    @TL_request
    def executeMacro(self, macro):
        '''Method that instructs the robot to run a macro.

        Args:
            macro (:py:class:`str`): Name of the macro to execute.
        '''
        # Logging the executeMacro command
        self.logger.info(f'Attempting to make robot execute macro "{macro}"')
        sendCommand(self, f'executeMacro {macro}')
    
    @TL_request
    def doTask(self, task, attributes):
        '''Method that instructs the robot to perform a task.

        Args:
            task (:py:class:`str`): Name of the task to do.

            attributes (:py:class:`str`): The attributes of the task.
        '''
        # Logging the doTask command
        self.logger.info(f'Attempting to make robot do task "{task} {attributes}"')
        sendCommand(self, f'doTask {task} {attributes}')

    @TL_request
    def dock(self):
        '''Method that instructs the robot to dock.
        '''
        # Logging the dock command
        self.logger.info('Attempting to dock the robot')
        sendCommand(self, 'dock')

    @TL_request
    def undock(self):
        '''Method that instructs the robot to undock.
        '''
        # Logging the undock command
        self.logger.info('Attempting to undock the robot')
        sendCommand(self, 'undock')
    
    @TL_request
    def stop(self):
        '''Method that instructs the robot to stop.
        '''
        # Logging the stop command
        self.logger.info('Attempting to stop the robot')
        sendCommand(self, 'stop')

    def __repr__(self):
        versions = self.datastore['Versions']
        identifier = self.config['Enterprise Manager Connection']['Identifier']
        status = self.status

        description = f'''
        Robot: {self.IP_address}
            ID:   {identifier}

            Status:
                Aram Status:         {status.aramStatus}
                State of Charge:     {status.stateOfCharge}

                Location:            X: {status.location[0]} [mm], Y: {status.location[1]} [mm], Θ: {status.location[2]} [deg]
                Localization Score:  {status.loc_score}

                CPU Temperature:     {status.temperature}

            SetNetGo Ver.:  {versions['SNG']}
            ARAM Ver.:      {versions['ARAM']}
            MARC Ver.:      {versions['MARC']}
        '''
        return inspect.cleandoc(description)

# %%
