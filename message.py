import json
import shortuuid

def as_message(dct):
     if '__message__' in dct:
         for id in dct['__message__']:
             for cmd in dct['__message__'][id]:
                 for param, args in dct['__message__'][id][cmd].items():
                    return Message(id, cmd, param, args)
     return dct

class MessageEncoder(json.JSONEncoder):
     def default(self, obj):
         if isinstance(obj, Message):
             return { '__message__': { msg.id : { msg.cmd : { msg.param : msg.args } } } }
         # Let the base class default method raise the TypeError
         return json.JSONEncoder.default(self, obj)

class Message():
    '''Base message class for passing commands through json'''

    def __init__(self, id, cmd, param, **kwargs):
        self.id = id if id is not None else shortuuid.uuid()
        self.cmd = cmd
        self.param = param
        self.args = kwargs if kwargs is not None else {}
            
