; Auto-generated. Do not edit!


(in-package sharedmem_transport-msg)


;//! \htmlinclude SharedMemMessage.msg.html

(defclass <SharedMemMessage> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (blockid
    :reader blockid-val
    :initarg :blockid
    :type integer
    :initform 0)
   (blocksize
    :reader blocksize-val
    :initarg :blocksize
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <SharedMemMessage>) ostream)
  "Serializes a message object of type '<SharedMemMessage>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'blockid)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'blockid)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'blockid)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'blockid)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'blocksize)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'blocksize)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'blocksize)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'blocksize)) ostream)
)
(defmethod deserialize ((msg <SharedMemMessage>) istream)
  "Deserializes a message object of type '<SharedMemMessage>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'blockid)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'blockid)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'blockid)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'blockid)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'blocksize)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'blocksize)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'blocksize)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'blocksize)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SharedMemMessage>)))
  "Returns string type for a message object of type '<SharedMemMessage>"
  "sharedmem_transport/SharedMemMessage")
(defmethod md5sum ((type (eql '<SharedMemMessage>)))
  "Returns md5sum for a message object of type '<SharedMemMessage>"
  "087f654de42a8c551e387e72726e40ed")
(defmethod message-definition ((type (eql '<SharedMemMessage>)))
  "Returns full string definition for message of type '<SharedMemMessage>"
  (format nil "Header header~%uint32 blockid~%uint32 blocksize~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <SharedMemMessage>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
))
(defmethod ros-message-to-list ((msg <SharedMemMessage>))
  "Converts a ROS message object to a list"
  (list '<SharedMemMessage>
    (cons ':header (header-val msg))
    (cons ':blockid (blockid-val msg))
    (cons ':blocksize (blocksize-val msg))
))
