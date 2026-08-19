# Task Module Overview

Tasks convert agent feedback into goals, obstacles, transforms, episode state,
and completion information. Policies consume the resulting `task_info` without
depending on the backend that produced the feedback.
