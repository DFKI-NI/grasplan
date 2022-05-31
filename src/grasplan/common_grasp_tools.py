#!/usr/bin/env python3

def remove_object_id(anchored_object_as_string):
    '''
    e.g. input  : relay_1
            output : relay
    '''
    if '_' not in anchored_object_as_string:
        return anchored_object_as_string
    count = 0
    for char in reversed(anchored_object_as_string):
        count += 1
        if char == '_':
            break
    return anchored_object_as_string[:-count]
