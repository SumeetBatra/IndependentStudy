from CCHTN import *

def main():
    parent_node = '0'
    activity_ids = ['1','2','3','4']

    TURTLE_CCHTN = CCHTN(parent_node)
    print(TURTLE_CCHTN.get_root_node()) #prints {'uid': '0', 'parent': None, 'skill': '0', 'completed': False, 'skillType': 'skill'}

    #Lines below don't work independently or together with code above
    #TURTLE_CCHTN.add_chain(activity_ids, parent_node) #TypeError: 'str' object does not support item assignment
    #TURTLE_CCHTN.add_subgraph(parent_node)
    #TURTLE_CCHTN.add_activity('1', parent_node)

if __name__ == "__main__":
    main()
