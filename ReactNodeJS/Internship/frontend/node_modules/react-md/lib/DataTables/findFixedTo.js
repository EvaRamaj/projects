'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.default = findFixedTo;
function findFixedTo(table) {
  if (table && table.firstChild.firstChild.classList.contains('md-data-table__scroll-wrapper')) {
    return {
      x: table,
      y: table.firstChild.firstChild
    };
  }

  return table;
}
//# sourceMappingURL=findFixedTo.js.map