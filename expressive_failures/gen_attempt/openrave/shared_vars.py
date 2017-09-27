import params
import openravepy

def iksolver(shared, Tgoal):
    """
    Returns a goal configuration and list of configruations using the IK solver. 
    We do not optimize for which configuration is returned yet.
    ---
    output: configuration and list of configurations
    """
    # generate the ik solver
    shared.robot.SetActiveManipulator(shared.manip)
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(shared.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    sol = shared.manip.FindIKSolution(Tgoal, openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
    sols = shared.manip.FindIKSolutions(Tgoal, openravepy.IkFilterOptions.IgnoreEndEffectorCollisions) # get collision-free solutions
    return sol,sols

class SharedVars:
    def __init__(self, task, env, robot):
        self.env = env
        self.robot = robot

        # Set up robot
        manip_left = self.robot.GetManipulator("leftarm")
        self.robot.SetDOFValues(params.left_arm_attitude, manip_left.GetArmIndices())
        self.manip_name = "rightarm"
        self.manip = self.robot.GetManipulator(self.manip_name)
        self.robot.SetActiveManipulator(self.manip)

        # Set up task
        self.task = task
        self.viewer = self.env.GetViewer()
        self.viewer.SetCamera(self.task["camera_array"])

        self.use_base = True
        if self.task["name"] in [params.LIFT["name"], params.PULL_DOWN["name"]]:
            self.use_base = False
        self.robot.SetDOFValues(self.task["default_starting_config"], self.manip.GetArmIndices())
        self.kin_bodies = []

        for t in range(len(self.task["target_transform"])):
            target = self.env.ReadKinBodyXMLFile(self.task["target"][t])
            self.env.AddKinBody(target) #add target
            self.kin_bodies.append(target)
            target.SetTransform(self.task["target_transform"][t])#set target transform

        for p in range(len(self.task["props"])): 
            prop = self.env.ReadKinBodyXMLFile(self.task["props"][p])
            self.env.AddKinBody(prop) #add props
            prop.SetTransform(self.task["props_transform"][p]) #set prop trasnforms

        #update Tgoal in dictionary 
        if self.task == params.LIFT:
            Tgoal = self.manip.GetEndEffectorTransform()
            Tgoal[:3][:,3][:2] = target.GetTransform()[:3][:,3][:2]
            Tgoal[:,3][2] = Tgoal[:,3][2] - 0.06
            self.task["Tgoal"] = Tgoal

        else:
            self.robot.SetDOFValues(self.task["default_starting_config"], self.manip.GetArmIndices())
            Tgoal = self.manip.GetEndEffectorTransform()
            self.task["Tgoal"] = Tgoal

        #use ik solver to solve for various starting configs and a goal config 
        self.starting_configs = []
        _,sols = iksolver(self, Tgoal)
        assert len(sols) > 0, "No solution found"  # No viable solutions for the starting xyz position

        self.robot.SetDOFValues(sols[0], self.manip.GetArmIndices()) #set dofs to first ik solution found; doesn't matter which ik solution we use
        goal_transform = self.manip.GetEndEffectorTransform()
        axis = self.task["axis"]
        vector = self.task["vector"]
        goal_transform[:3][:,3][axis] += vector  # update goal_transform accordingly

        self.goal_config_stationary,_ = iksolver(self, goal_transform) #get ik solution for goal_transform
        assert self.goal_config_stationary is not None, "No goal solution found"
        self.robot.SetDOFValues(self.goal_config_stationary, self.manip.GetArmIndices())
        tr = self.manip.GetEndEffectorTransform()
        assert (goal_transform-tr<1e-10).all() #make sure this ik solution's xyz position is approximately equal to goal_transform's xyz

        for sol in sols: #make sure the xyz positions for tall the starting position ik solutions are approximately equal to Tgoal' xyz
            self.robot.SetDOFValues(sol, self.manip.GetArmIndices())
            trans = self.manip.GetEndEffectorTransform()
            assert (trans-Tgoal < 1e-9).all()
            self.starting_configs.append(sol)

        #open gripper
        self.robot.SetDOFValues([0],[34]) #set default gripper to closed 
