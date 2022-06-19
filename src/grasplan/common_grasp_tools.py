#!/usr/bin/env python3

def separate_object_class_from_id(anchored_object_as_string):
    '''
    e.g. input  : relay_1
            output : relay, 1
    '''
    if '_' not in anchored_object_as_string:
        return anchored_object_as_string, None
    count = 0
    for char in reversed(anchored_object_as_string):
        count += 1
        if char == '_':
            break
    chars_before_number = anchored_object_as_string[:-count]
    try:
        object_id = int(anchored_object_as_string[-count+1:])
        return chars_before_number, object_id
    except:
        return anchored_object_as_string, None

class objectToPick():
    def __init__(self, obj_class_and_id_as_string=None):
        '''
        this class is meant to be used as a struct that holds info on:
        object class, id, and a boolean to represent whether any obj should be picked.
        self.any_obj_id : if true it means any object_id can be picked
        '''
        self.obj_class = None
        self.id = None
        self.any_obj_id = None
        if not obj_class_and_id_as_string is None:
            assert isinstance(obj_class_and_id_as_string, str)
            self.obj_class, self.id = separate_object_class_from_id(obj_class_and_id_as_string)
            if self.id is None:
                self.any_obj_id = True
            else:
                self.any_obj_id = False

    def set_object_class(self, obj_class):
        assert isinstance(obj_class, str)
        self.obj_class = obj_class

    def set_id(self, id):
        assert isinstance(id, int)
        if id is None:
            self.any_obj_id = True
            return
        self.id = str(id)
        self.object_name = self.obj_class + '_' + self.id

    def set_any_object(self, value):
        assert isinstance(value, bool)
        self.any_obj_id = value

    def get_object_class_and_id_as_string(self):
        assert self.obj_class is not None
        if self.id is None:
            return self.obj_class
        else:
            return self.obj_class + '_' + str(self.id)

    def get_all(self):
        return self.obj_class, self.id, self.any_obj_id
