from CCHTN import *
import networkx as nx

class Classifier(object):
    def __init__(self):

def main():
    parent_node = 'task_graph'
    activity_ids = ['goal_1','goal_2','goal_3','goal_4']

    TURTLE_CCHTN = CCHTN(parent_node)
    print(TURTLE_CCHTN.get_root_node()) #prints {'uid': '0', 'parent': None, 'skill': '0', 'completed': False, 'skillType': 'skill'}
    TURTLE_CCHTN.add_chain(activity_ids, TURTLE_CCHTN.get_root_node())


if __name__ == "__main__":
    main()
