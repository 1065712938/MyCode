
(cl:in-package :asdf)

(defsystem "custom_msg_topic-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "custom_msg" :depends-on ("_package_custom_msg"))
    (:file "_package_custom_msg" :depends-on ("_package"))
  ))