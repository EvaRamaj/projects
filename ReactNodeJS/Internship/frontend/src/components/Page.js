"use strict";

import React from 'react';

import Header from './Header';
import { Footer } from './Footer';

export default class Page extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            title: '',
            isSticky: !!this.props.isStickyFooter
        }
    }

    componentDidMount(){
       this.setState({
           title: document.title
       });
    }

    render() {
        return (
            <section>
                <Header title={this.state.title} />
                {this.props.children}
                <Footer isSticky={this.state.isSticky}/>
            </section>
        );
    }
}