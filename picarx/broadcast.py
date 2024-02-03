from readerwriterlock import rwlock

class broadcast:

    def __init__(self):
        self.message = None  # Initialize the message attribute to None
        self.lock = rwlock.RWLockWrite() # Initialize the read-write lock with writer priority

    def write(self, message):
        
        with self.lock.gen_wlock():  # Acquire a write lock
            self.message = message  # Set the message attribute to the new message

    def read(self):
        
        with self.lock.gen_rlock():  # Acquire a read lock
            return self.message  # Return the current message