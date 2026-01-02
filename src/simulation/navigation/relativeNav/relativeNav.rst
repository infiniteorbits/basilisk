Executive Summary
-----------------

Relative navigation model used to provide error-ed truth (or truth). This class is used to perturb the truth state away using a gauss-markov
error model, but making the error proportional to the relative distance norm. It is designed to look like a random walk process put on top of
the nominal relative position, velocity, attitude, and attitude rate.  This is meant to
be used in place of the nominal relative navigation system output.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - RelAttOutMsg
      - :ref:`RelNavAttMsgPayload`
      - relative attitude navigation output msg
    * - RelTransOutMsg
      - :ref:`RelNavTransMsgPayload`
      - relative translation navigation output msg
    * - servicerStateInMsg
      - :ref:`SCStatesMsgPayload`
      - servicer spacecraft state input msg
    * - clientStateInMsg
      - :ref:`SCStatesMsgPayload`
      - client spacecraft state input msg