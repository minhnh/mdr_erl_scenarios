import actionlib
import rospy
from rosplan_knowledge_msgs.srv import GetAttributeServiceRequest
from atwork_ros_msgs.msg import BenchmarkState, BenchmarkFeedback
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mcr_perception_msgs.msg import Object, DetectSceneAction


class GetBenchmarkState(ScenarioStateBase):
    _state_sub = None   # type: rospy.Subscriber
    _timeout = None     # type: rospy.Duration
    _bm_state = None    # type: BenchmarkState

    def __init__(self, **kwargs):
        ScenarioStateBase.__init__(self, 'get_benchmark_state',
                                   outcomes=['running_calibration', 'running_preparation', 'running_execution',
                                             'paused', 'finished', 'stopped'],
                                   output_keys=['benchmark_state'], **kwargs)
        self._state_sub = rospy.Subscriber("/robot_example_ros/benchmark_state",
                                           BenchmarkState, self._benchmark_state_cb, queue_size=1)
        self._timeout = rospy.Duration.from_sec(kwargs.get('timeout', 5.0))

    def _benchmark_state_cb(self, bm_state_msg):
        self._bm_state = bm_state_msg

    def execute(self, ud):
        self._bm_state = None
        rospy.loginfo('Asking for benchmark state')

        # wait for new benchmark message
        start_time = rospy.Time.now()
        while self._bm_state is None:
            if (rospy.Time.now() - start_time) > self._timeout:
                rospy.logerr('Timed out waiting for benchmark state')
                return 'paused'
            rospy.sleep(0.1)

        # read benchmark message
        ud.benchmark_state = self._bm_state
        if self._bm_state.state.data == BenchmarkState.RUNNING:
            if self._bm_state.phase.data == BenchmarkState.CALIBRATION:
                rospy.loginfo('state: running_calibration')
                return 'running_calibration'

            if self._bm_state.phase.data == BenchmarkState.PREPARATION:
                rospy.loginfo("state: running_preparation")
                return 'running_preparation'

            if self._bm_state.phase.data == BenchmarkState.EXECUTION:
                rospy.loginfo("state: running_execution")
                return 'running_execution'

            rospy.logerr('Unexpected running phase: {1}'.format(self.state_name, self._bm_state.phase.data))
            return 'paused'

        if self._bm_state.state.data == BenchmarkState.PAUSED:
            rospy.loginfo('state: paused')
            return 'paused'

        if self._bm_state.state.data == BenchmarkState.STOPPED:
            rospy.loginfo('state: stopped')
            return 'stopped'

        if self._bm_state.state.data == BenchmarkState.FINISHED:
            rospy.loginfo('state: finished')
            return 'finished'

        rospy.logerr('unexpected state: ' + self._bm_state.state.data)
        return 'paused'

    def get_dispatch_msg(self):
        pass


class SendBenchmarkFeedback(ScenarioStateBase):
    _detection_client = None    # type: actionlib.SimpleActionClient
    _timeout = None             # type: rospy.Duration
    _benchmark_fb_pub = None    # type: rospy.Publisher
    _scenario = None            # type: str
    _phase = None               # type: str
    _retry_num = None           # type: int

    def __init__(self, scenario, phase, **kwargs):
        outcomes = kwargs.get('outcomes', ['done', 'timeout', 'retry', 'failed'])
        ScenarioStateBase.__init__(self, 'send_benchmark_feedback',
                                   outcomes=outcomes, input_keys=['benchmark_state'], **kwargs)

        self._scenario = scenario
        self._phase = phase
        self._timeout = rospy.Duration.from_sec(kwargs.get('timeout', 20.0))
        self._retry_num = kwargs.get('number_of_retries', 0)

        # TODO: hack solution until skill status is implemented in the knowledge base
        self._detection_client = actionlib.SimpleActionClient('/mas_perception/detect_image', DetectSceneAction)
        self._benchmark_fb_pub = rospy.Publisher("/robot_example_ros/benchmark_feedback", BenchmarkFeedback)

    def _get_surface_objects(self, surface_prefix):
        surface_objects = dict()
        request = GetAttributeServiceRequest()
        request.predicate_name = 'on'
        result = self.attribute_fetching_client(request)
        for item in result.attributes:
            object_on_desired_surface = False
            object_name = ''
            surface_name = ''
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'plane' and param.value.find(surface_prefix) != -1:
                        object_on_desired_surface = True
                        surface_name = param.value
                        if surface_name not in surface_objects:
                            surface_objects[surface_name] = list()
                    elif param.key == 'obj':
                        object_name = param.value
            if object_on_desired_surface:
                surface_objects[surface_name].append(object_name)
        return surface_objects

    def _get_first_object(self, surface_object_dict):
        if len(surface_object_dict) == 0:
            return None

        first_object = None
        for surface, objects in surface_object_dict.items():
            rospy.loginfo('found plane: ' + surface)
            for obj_name in objects:
                try:
                    first_object = self.msg_store_client.query_named(obj_name, Object._type)[0]
                except Exception as e:
                    rospy.logerr('Error retrieving knowledge about object "{0}": {1}'.format(obj_name, e.message))
                    pass

                # break for loop objects
                if first_object is not None:
                    rospy.loginfo('found first object: ' + obj_name)
                    break

            # break for loop surfaces
            if first_object is not None:
                break

        return first_object

    def _check_retry_num(self, fail_state='failed'):
        if self._retry_num > 0:
            self._retry_num -= 1
            rospy.logwarn('retrying...')
            return 'retry'
        return fail_state

    def _wait_for_perception_functions(self):
        return self._detection_client.wait_for_server(self._timeout)

    def _send_fbm1_feedback(self):
        if self._phase == 'prep':
            if not self._wait_for_perception_functions():
                return self._check_retry_num(fail_state='timeout')

            fb_msg = BenchmarkFeedback()
            fb_msg.phase_to_terminate = BenchmarkFeedback.PREPARATION
            self._benchmark_fb_pub.publish(fb_msg)
            return 'done'

        if self._phase == 'exec':
            # get first object and send feedback
            surface_object_dict = self._get_surface_objects('table')
            first_object = self._get_first_object(surface_object_dict)

            if first_object is None:
                rospy.logwarn('found no objects')
                return self._check_retry_num()

            fb_msg = BenchmarkFeedback()
            fb_msg.phase_to_terminate = BenchmarkFeedback.EXECUTION
            # TODO: confirm behavior here
            fb_msg.object_class_name = first_object.name
            fb_msg.object_instance_name = first_object.name
            self._benchmark_fb_pub.publish(fb_msg)
            return 'done'

        rospy.logerr('unrecognized FBM1 phase: ' + self._phase)
        return self._check_retry_num()

    def execute(self, _):
        if self._scenario == 'fbm1':
            return self._send_fbm1_feedback()

        rospy.logerr('unrecognized scenario: ' + self._scenario)
        return self._check_retry_num()

    def get_dispatch_msg(self):
        pass

