'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.default = isInvalidAnimate;
var gcc = 'getComposedComponent';

/**
 * Just checks if the fab is actually a ref to the FAB Button.
 */
function isInvalidAnimate(fab) {
  return typeof fab[gcc] !== 'function' || typeof fab[gcc]()[gcc] !== 'function' || typeof fab[gcc]()[gcc]()._animateForSnackbar !== 'function';
}
//# sourceMappingURL=isInvalidAnimate.js.map