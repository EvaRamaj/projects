"use strict";

import React from 'react';

import { ItemEvaluationList } from '../components/ItemEvaluationList';

import ItemEvaluationService from '../services/ItemEvaluationService';


export class ItemEvaluationListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: true,
            data: []
        };
    }

    componentWillMount(){
        this.setState({
            loading: false
        });

        ItemEvaluationService.getItemEvaluations().then((data) => {
            console.log("lasme")
            console.log(data)
            console.log("lasme")
            this.setState({
                data: [...data.evaluations],
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }


    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (

            <ItemEvaluationList data={this.state.data} onDelete={(id) => this.deleteItemEvaluation(id)}/>
        );
    }
}
