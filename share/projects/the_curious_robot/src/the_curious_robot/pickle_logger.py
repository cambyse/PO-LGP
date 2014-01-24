import datetime
import os
import pickle


class pickle_member(object):
    """
    A simple decorator that pickles the given instance variable
    `member_name` after calling the decorated function.

    Just decorate a function and supply the member_name.  Folders are
    automatically created.
    """
    def __init__(self, name, folder):
        self.member_name = name

        # Create a folder for the logs
        pattern = "log_tcr/{}/%Y-%m-%d %H:%M:%S/{}/".format(folder,
                                                            self.member_name)
        self.foldername = datetime.datetime.now().strftime(pattern)
        print "Initializing pickler for {}; will be saved in {}.".format(
            self.member_name, self.foldername)
        try:
            os.makedirs(self.foldername)
        except OSError:
            print "%s already exists.".format(self.foldername)

    def __call__(self, func):
        def wrapped(*args, **kwargs):
            result = func(*args, **kwargs)
            # pickle the data
            self._pickle(decorated_class=args[0])
            return result
        return wrapped

    def _pickle(self, decorated_class):
        """The pickling logic"""
        # construct filename for pkl file
        filename = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f.pkl")
        filename = os.path.join(self.foldername, filename)
        print "Pickling {}".format(filename)

        # pickle the instance variable
        member = getattr(decorated_class, self.member_name)
        with open(filename, "w") as f:
            pickle.dump(member, f)
