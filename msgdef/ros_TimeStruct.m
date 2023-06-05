function msg = ros_TimeStruct
% Message struct definition for ros/Time
coder.inline("never")
msg = struct(...
    'Sec',ros.internal.ros.messages.ros.default_type('uint32',1),...
    'Nsec',ros.internal.ros.messages.ros.default_type('uint32',1));
coder.cstructname(msg,'ros_TimeStruct_T');
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
