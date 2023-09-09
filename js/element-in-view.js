(function($) {

  /**
   * Copyright 2012, Digital Fusion
   * Licensed under the MIT license.
   * http://teamdf.com/jquery-plugins/license/
   *
   * @author Sam Sehnert
   * @desc A small plugin that checks whether elements are within
   *     the user visible viewport of a web browser.
   *     only accounts for vertical position, not horizontal.
   */

  $.fn.visible = function(partial) {
    
      var $t            = $(this),
          $w            = $(window),
          viewTop       = $w.scrollTop(),
          viewBottom    = viewTop + $w.height(),
          _top          = $t.offset().top,
          _bottom       = _top + $t.height(),
          compareTop    = partial === true ? _bottom : _top,
          compareBottom = partial === true ? _top : _bottom;
    
    return ((compareBottom <= viewBottom) && (compareTop >= viewTop));

  };
    
})(jQuery);

$(window).on('scroll', function() {
  
  $(".about-us .title h2,.about-us .text-box,.sec-title,.about-section-two .content-column h2,.footer-logo img,.portfolio-item-full-width .content-column h2,.about-section-two .image-column .inner-column,.page-title .drops-icon,.page-title h1 span,.portfolio-single.style-two .text-column .inner-column,.portfolio-single.style-two .info-column .inner-column,.page-title .drop-icon-small,.page-title .drop-icon,.error-page-section .icon-2,.error-page-section .icon-1,.error-section .btn-style-one,.page-title .drops-icon").each(function(i, el) {
    var el = $(el);
    if (el.visible(true)) {
      el.addClass("now-in-view"); 
    } else {
      el.removeClass("now-in-view");
    }
  });
  
});

$(document).on('ready', function() {
  $(".about-us .title h2,.about-us .text-box,.sec-title,.about-section-two .content-column h2,.footer-logo img,.portfolio-item-full-width .content-column h2,.about-section-two .image-column .inner-column,.page-title .drops-icon,.page-title h1 span,.portfolio-single.style-two .text-column .inner-column,.portfolio-single.style-two .info-column .inner-column,.page-title .drop-icon-small,.page-title .drop-icon,.error-page-section .icon-2,.error-page-section .icon-1,.error-section .btn-style-one,.page-title .drops-icon").each(function(i, el) {
	var el = $(el);
	if (el.visible(true)) {
	  el.addClass("now-in-view"); 
	} else {
	  el.removeClass("now-in-view");
	}
  });
});