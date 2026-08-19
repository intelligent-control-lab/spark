document.addEventListener("DOMContentLoaded", () => {
  const tree = document.querySelector(".sidebar-tree");
  if (!tree) return;

  for (const item of tree.querySelectorAll("li.toctree-l1, li.toctree-l2")) {
    const checkbox = item.querySelector(":scope > input.toctree-checkbox");
    if (checkbox) checkbox.checked = true;
  }

  for (const item of tree.querySelectorAll("li.toctree-l3")) {
    const checkbox = item.querySelector(":scope > input.toctree-checkbox");
    if (checkbox && !item.classList.contains("current")) checkbox.checked = false;
  }
});
